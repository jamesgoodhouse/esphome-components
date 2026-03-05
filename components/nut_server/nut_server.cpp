#include "nut_server.h"
#include "../ups_hid/ups_hid.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/hal.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#ifdef USE_ESP32
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <fcntl.h>
#include <errno.h>
#endif

namespace esphome {
namespace nut_server {

NutServerComponent::NutServerComponent() {
  clients_.reserve(DEFAULT_MAX_CLIENTS);
}

NutServerComponent::~NutServerComponent() {
  stop_server();
}

void NutServerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NUT Server...");

  if (!ups_hid_) {
    ESP_LOGE(TAG, "No UPS HID component configured!");
    mark_failed();
    return;
  }

  // Initialize clients
  clients_.resize(max_clients_);
  for (auto &client : clients_) {
    client.reset();
  }

  if (!start_server()) {
    ESP_LOGE(TAG, "Failed to start NUT server!");
    mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "NUT Server started on port %d", port_);
}

void NutServerComponent::loop() {
  // All work happens in server_task; nothing to do on the main loop
}

void NutServerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "NUT Server:");
  ESP_LOGCONFIG(TAG, "  Port: %d", port_);
  ESP_LOGCONFIG(TAG, "  Max Clients: %d", max_clients_);
  ESP_LOGCONFIG(TAG, "  UPS Name: %s", get_ups_name().c_str());
  ESP_LOGCONFIG(TAG, "  Authentication: %s", auth_enabled() ? "Enabled" : "Disabled");
  for (const auto &user : users_) {
    ESP_LOGCONFIG(TAG, "    User: %s", user.username.c_str());
  }

  if (ups_hid_) {
    ESP_LOGCONFIG(TAG, "  UPS HID Component: Connected");
  } else {
    ESP_LOGCONFIG(TAG, "  UPS HID Component: Not configured!");
  }
}

bool NutServerComponent::start_server() {
#ifdef USE_ESP32
  // Create server socket
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    return false;
  }

  // Set socket options
  int yes = 1;
  if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
    ESP_LOGW(TAG, "Failed to set SO_REUSEADDR: %d", errno);
  }

  // Set non-blocking mode
  int flags = fcntl(server_socket_, F_GETFL, 0);
  if (fcntl(server_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    ESP_LOGW(TAG, "Failed to set non-blocking mode: %d", errno);
  }

  // Bind to port
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(port_);

  if (bind(server_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind to port %d: %d", port_, errno);
    close(server_socket_);
    server_socket_ = -1;
    return false;
  }

  // Start listening
  if (listen(server_socket_, max_clients_) < 0) {
    ESP_LOGE(TAG, "Failed to listen on socket: %d", errno);
    close(server_socket_);
    server_socket_ = -1;
    return false;
  }

  // Create server task
  server_running_ = true;
  server_task_exited_ = false;
  xTaskCreate(server_task, "nut_server", 8192, this, 1, &server_task_handle_);

  return true;
#else
  ESP_LOGE(TAG, "NUT Server requires ESP32 platform");
  return false;
#endif
}

void NutServerComponent::stop_server() {
#ifdef USE_ESP32
  shutdown_requested_ = true;
  server_running_ = false;

  // Close all client connections
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto &client : clients_) {
      if (client.is_active()) {
        disconnect_client(client);
      }
    }
  }

  // Close server socket
  if (server_socket_ >= 0) {
    close(server_socket_);
    server_socket_ = -1;
  }

  // Wait for server task to exit on its own (max 5s)
  if (server_task_handle_) {
    for (int i = 0; i < 50 && !server_task_exited_.load(); i++) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!server_task_exited_.load()) {
      ESP_LOGW(TAG, "Server task did not exit in time, force deleting");
      vTaskDelete(server_task_handle_);
    }
    server_task_handle_ = nullptr;
  }
#endif
}

void NutServerComponent::server_task(void *param) {
#ifdef USE_ESP32
  NutServerComponent *server = static_cast<NutServerComponent *>(param);

  while (server->server_running_) {
    server->accept_clients();

    // Handle all connected clients and clean up inactive ones
    {
      std::lock_guard<std::mutex> lock(server->clients_mutex_);

      uint32_t now = millis();
      for (auto &client : server->clients_) {
        if (!client.is_active())
          continue;

        // Check for inactive timeout
        if ((now - client.last_activity) > CLIENT_TIMEOUT_MS) {
          ESP_LOGD(TAG, "Client %s timed out after %lus, disconnecting",
                   client.remote_ip.c_str(),
                   (unsigned long)((now - client.last_activity) / 1000));
          server->disconnect_client(client);
          continue;
        }

        server->handle_client(client);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  server->server_task_exited_.store(true);
  vTaskDelete(nullptr);
#endif
}

void NutServerComponent::accept_clients() {
#ifdef USE_ESP32
  // Drain all pending connections from the accept queue
  for (;;) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    int client_socket = accept(server_socket_, (struct sockaddr *)&client_addr, &client_len);
    if (client_socket < 0) {
      if (errno != EWOULDBLOCK && errno != EAGAIN) {
        ESP_LOGW(TAG, "Accept failed: %d", errno);
      }
      return;  // No more pending connections
    }

    // Set client socket to non-blocking
    int flags = fcntl(client_socket, F_GETFL, 0);
    fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);

    // Enable TCP keepalive to detect dead connections faster
    int keepAlive = 1;
    setsockopt(client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(keepAlive));
#if defined(TCP_KEEPIDLE) && defined(TCP_KEEPINTVL) && defined(TCP_KEEPCNT)
    int keepIdle = 10;      // Start probes after 10 seconds idle
    int keepInterval = 5;   // Probe every 5 seconds
    int keepCount = 3;      // Disconnect after 3 failed probes
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(keepIdle));
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(keepInterval));
    setsockopt(client_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(keepCount));
#endif

    // Find available client slot
    std::lock_guard<std::mutex> lock(clients_mutex_);
    bool assigned = false;
    for (auto &client : clients_) {
      if (!client.is_active()) {
        client.socket_fd = client_socket;
        client.state = ClientState::CONNECTED;
        uint32_t now = millis();
        client.last_activity = now;
        client.connect_time = now;
        client.login_attempts = 0;
        client.remote_ip = std::string(inet_ntoa(client_addr.sin_addr));

        ESP_LOGD(TAG, "Client connected from %s", client.remote_ip.c_str());
        assigned = true;
        break;
      }
    }

    if (!assigned) {
      // Count active clients and their IPs for diagnostics
      uint8_t active = 0;
      std::string active_ips;
      for (const auto &c : clients_) {
        if (c.is_active()) {
          active++;
          if (!active_ips.empty()) active_ips += ", ";
          active_ips += c.remote_ip;
        }
      }
      ESP_LOGW(TAG, "Maximum clients (%d/%d) reached, rejecting from %s. Active: [%s]",
               active, max_clients_, inet_ntoa(client_addr.sin_addr), active_ips.c_str());
      const char *msg = "ERR MAX-CLIENTS Maximum number of clients reached\n";
      send(client_socket, msg, strlen(msg), 0);
      close(client_socket);
    }
  }
#endif
}

void NutServerComponent::handle_client(NutClient &client) {
#ifdef USE_ESP32
  char buffer[MAX_COMMAND_LENGTH];
  int bytes_received = recv(client.socket_fd, buffer, sizeof(buffer) - 1, 0);

  if (bytes_received > 0) {
    buffer[bytes_received] = '\0';

    // Remove trailing newline
    char *newline = strchr(buffer, '\n');
    if (newline) *newline = '\0';
    newline = strchr(buffer, '\r');
    if (newline) *newline = '\0';

    client.last_activity = millis();

    ESP_LOGV(TAG, "Received command: %s", buffer);
    process_command(client, std::string(buffer));

  } else if (bytes_received == 0) {
    // Client disconnected cleanly
    ESP_LOGD(TAG, "Client %s disconnected", client.remote_ip.c_str());
    disconnect_client(client);
  } else {
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      if (errno == ECONNRESET || errno == EPIPE) {
        ESP_LOGD(TAG, "Client %s connection reset (error %d)", client.remote_ip.c_str(), errno);
      } else {
        ESP_LOGW(TAG, "Receive error %d from %s", errno, client.remote_ip.c_str());
      }
      disconnect_client(client);
    }
  }
#endif
}

void NutServerComponent::disconnect_client(NutClient &client) {
#ifdef USE_ESP32
  if (client.socket_fd >= 0) {
    close(client.socket_fd);
  }
  client.reset();
#endif
}

void NutServerComponent::process_command(NutClient &client, const std::string &command) {
  if (command.empty()) {
    return;
  }

  // Parse command and arguments
  size_t space_pos = command.find(' ');
  std::string cmd = (space_pos != std::string::npos) ?
                     command.substr(0, space_pos) : command;
  std::string args = (space_pos != std::string::npos) ?
                      command.substr(space_pos + 1) : "";

  // Convert command to uppercase for comparison
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);

  // Handle STARTTLS silently (sent on every connection by most clients)
  if (cmd == "STARTTLS") {
    handle_starttls(client);
    return;
  }

  // Log all other commands
  ESP_LOGD(TAG, "Received command: '%s' args: '%s'", cmd.c_str(), args.c_str());

  // Commands that don't require authentication
  if (cmd == "HELP") {
    handle_help(client);
  } else if (cmd == "VER" || cmd == "VERSION") {
    handle_version(client);
  } else if (cmd == "NETVER") {
    handle_netver(client);
  } else if (cmd == "USERNAME") {
    handle_username(client, args);
  } else if (cmd == "PASSWORD") {
    handle_password(client, args);
  } else if (cmd == "LOGIN") {
    handle_login(client, args);
  } else if (cmd == "LOGOUT") {
    handle_logout(client);
  } else if (cmd == "UPSDVER") {
    handle_upsdver(client);
  }
  // Read-only commands: LIST and GET do NOT require authentication per NUT protocol.
  // Any NUT client (including monitoring tools like Synology) can query UPS status
  // without providing credentials.
  else if (cmd == "LIST") {
    size_t sub_pos = args.find(' ');
    std::string subcmd = (sub_pos != std::string::npos) ?
                         args.substr(0, sub_pos) : args;
    std::string subargs = (sub_pos != std::string::npos) ?
                          args.substr(sub_pos + 1) : "";

    std::transform(subcmd.begin(), subcmd.end(), subcmd.begin(), ::toupper);

    if (subcmd == "UPS") {
      handle_list_ups(client);
    } else if (subcmd == "VAR") {
      handle_list_var(client, subargs);
    } else if (subcmd == "CMD") {
      handle_list_cmd(client, subargs);
    } else if (subcmd == "CLIENTS") {
      handle_list_clients(client);
    } else if (subcmd == "RW") {
      handle_list_rwvar(client, subargs);
    } else if (subcmd == "ENUM") {
      handle_list_enum(client, subargs);
    } else if (subcmd == "RANGE") {
      handle_list_range(client, subargs);
    } else {
      send_error(client, "INVALID-ARGUMENT");
    }
  } else if (cmd == "GET") {
    size_t sub_pos = args.find(' ');
    std::string subcmd = (sub_pos != std::string::npos) ?
                         args.substr(0, sub_pos) : args;
    std::string subargs = (sub_pos != std::string::npos) ?
                          args.substr(sub_pos + 1) : "";

    std::transform(subcmd.begin(), subcmd.end(), subcmd.begin(), ::toupper);

    if (subcmd == "VAR") {
      handle_get_var(client, subargs);
    } else if (subcmd == "NUMLOGINS") {
      handle_get_numlogins(client, subargs);
    } else if (subcmd == "UPSDESC") {
      handle_get_upsdesc(client, subargs);
    } else if (subcmd == "DESC") {
      handle_get_desc(client, subargs);
    } else if (subcmd == "TYPE") {
      handle_get_type(client, subargs);
    } else {
      send_error(client, "INVALID-ARGUMENT");
    }
  } else if (cmd == get_ups_name()) {
    // Legacy upsc -l format: sends UPS name directly as command
    handle_legacy_list_vars(client, cmd);
  }
  // Write commands: SET, INSTCMD, FSD require authentication
  else if (auth_enabled() && !client.is_authenticated()) {
    send_error(client, "ACCESS-DENIED");
  } else if (cmd == "SET") {
    size_t sub_pos = args.find(' ');
    std::string subcmd = (sub_pos != std::string::npos) ?
                         args.substr(0, sub_pos) : args;
    std::string subargs = (sub_pos != std::string::npos) ?
                          args.substr(sub_pos + 1) : "";

    std::transform(subcmd.begin(), subcmd.end(), subcmd.begin(), ::toupper);

    if (subcmd == "VAR") {
      handle_set_var(client, subargs);
    } else {
      send_error(client, "INVALID-ARGUMENT");
    }
  } else if (cmd == "INSTCMD") {
    handle_instcmd(client, args);
  } else if (cmd == "FSD") {
    handle_fsd(client, args);
  } else {
    ESP_LOGW(TAG, "Unknown command received: '%s' with args: '%s'", cmd.c_str(), args.c_str());
    send_error(client, "UNKNOWN-COMMAND");
  }
}

void NutServerComponent::handle_login(NutClient &client, const std::string &args) {
  auto parts = split_args(args);

  // Standard NUT LOGIN flow: LOGIN <upsname>
  // This is called after USERNAME and PASSWORD commands have been processed.
  // The client should already be authenticated via the PASSWORD handler.

  // If client is already authenticated (via USERNAME/PASSWORD flow), just accept LOGIN
  if (client.state == ClientState::AUTHENTICATED) {
    // Verify UPS name if provided
    if (parts.size() == 1 && parts[0] != get_ups_name()) {
      send_error(client, "UNKNOWN-UPS");
      return;
    }
    send_response(client, "OK\n");
    ESP_LOGD(TAG, "Client login accepted for UPS %s", get_ups_name().c_str());
    return;
  }

  // Standard NUT protocol: LOGIN <upsname> (1 argument)
  // Authentication should have been done via USERNAME + PASSWORD commands
  if (parts.size() == 1) {
    // If no authentication is configured, allow login without credentials
    if (!auth_enabled()) {
      client.state = ClientState::AUTHENTICATED;
      send_response(client, "OK\n");
      ESP_LOGD(TAG, "Client login accepted (no auth required) for UPS %s", parts[0].c_str());
      return;
    }

    // Password is required but client hasn't authenticated
    // Check if temp credentials were provided via USERNAME/PASSWORD
    if (!client.temp_username.empty() && !client.temp_password.empty()) {
      if (authenticate(client.temp_username, client.temp_password)) {
        client.state = ClientState::AUTHENTICATED;
        client.username = client.temp_username;
        client.temp_username.clear();
        client.temp_password.clear();
        send_response(client, "OK\n");
        ESP_LOGD(TAG, "Client %s authenticated via LOGIN as %s", client.remote_ip.c_str(), client.username.c_str());
        return;
      }
    }

    send_error(client, "ACCESS-DENIED");
    return;
  }

  // Legacy format: LOGIN <username> <password> (2 arguments, non-standard)
  if (parts.size() == 2) {
    if (authenticate(parts[0], parts[1])) {
      client.state = ClientState::AUTHENTICATED;
      client.username = parts[0];
      send_response(client, "OK\n");
      ESP_LOGD(TAG, "Client %s authenticated as %s (legacy login)", client.remote_ip.c_str(), parts[0].c_str());
    } else {
      client.login_attempts++;
      if (client.login_attempts >= MAX_LOGIN_ATTEMPTS) {
        ESP_LOGW(TAG, "Max login attempts exceeded, disconnecting client");
        disconnect_client(client);
      } else {
        send_error(client, "ACCESS-DENIED");
      }
    }
    return;
  }

  send_error(client, "INVALID-ARGUMENT");
}

void NutServerComponent::handle_logout(NutClient &client) {
  send_response(client, "OK Goodbye\n");
  // Close the connection after sending the goodbye message
  disconnect_client(client);
}

void NutServerComponent::handle_list_ups(NutClient &client) {
  std::string ups_name = get_ups_name();
  std::string ups_description = get_ups_description();

  std::string response = "BEGIN LIST UPS\n";
  response += "UPS " + ups_name + " \"" + ups_description + "\"\n";
  response += "END LIST UPS\n";
  send_response(client, response);
}

void NutServerComponent::handle_list_var(NutClient &client, const std::string &args) {
  if (args != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  if (!has_ups_data() || !ups_hid_) {
    send_error(client, "DATA-STALE");
    return;
  }

  // Single snapshot: one mutex acquisition instead of hundreds.
  auto snapshot = ups_hid_->get_ups_data();

  std::string ups_name = get_ups_name();
  std::string response = "BEGIN LIST VAR " + ups_name + "\n";

  std::vector<std::string> variables = {
    "battery.capacity", "battery.charge", "battery.charge.low", "battery.charge.warning",
    "battery.mfr.date", "battery.runtime", "battery.type",
    "battery.voltage", "battery.voltage.low", "battery.voltage.nominal",
    "device.mfr", "device.model", "device.serial", "device.type",
    "driver.name", "driver.version", "driver.version.internal",
    "input.frequency", "input.voltage", "input.voltage.nominal",
    "input.transfer.low", "input.transfer.high",
    "output.current", "output.frequency", "output.frequency.nominal",
    "output.voltage", "output.voltage.nominal",
    "ups.beeper.status", "ups.delay.shutdown",
    "ups.firmware", "ups.load",
    "ups.mfr", "ups.model",
    "ups.power.nominal", "ups.realpower", "ups.realpower.nominal",
    "ups.serial", "ups.status", "ups.test.result",
    "ups.timer.reboot", "ups.timer.shutdown",
    "ups.debug.read.status",
    "ups.debug.reset.reason",
    "ups.debug.event.count",
  };

  for (const auto &var : variables) {
    std::string value = resolve_ups_var(var, snapshot);
    if (!value.empty()) {
      response += "VAR " + ups_name + " " + var + " \"" + value + "\"\n";
    }
  }

  // Append individual event log entries
  size_t event_count = ups_hid_->get_event_log().size();
  for (size_t i = 0; i < event_count; i++) {
    std::string event = ups_hid_->get_event_log().get_event(i);
    if (!event.empty()) {
      response += "VAR " + ups_name + " ups.debug.event." + std::to_string(i) + " \"" + event + "\"\n";
    }
  }

  response += "END LIST VAR " + ups_name + "\n";
  ESP_LOGD(TAG, "LIST VAR response: %zu bytes, %zu variables", response.length(),
           std::count(response.begin(), response.end(), '\n') - 2);
  send_response(client, response);
}

void NutServerComponent::handle_get_var(NutClient &client, const std::string &args) {
  auto parts = split_args(args);
  if (parts.size() != 2) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  if (parts[0] != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  std::string value = get_ups_var(parts[1]);
  if (!value.empty()) {
    std::string response = "VAR " + get_ups_name() + " " + parts[1] + " \"" + value + "\"\n";
    send_response(client, response);
  } else {
    send_error(client, "VAR-NOT-SUPPORTED");
  }
}

void NutServerComponent::handle_get_numlogins(NutClient &client, const std::string &args) {
  if (args != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  // Count authenticated clients that have sent LOGIN for this UPS
  int num_logins = 0;
  for (const auto &c : clients_) {
    if (c.is_authenticated()) {
      num_logins++;
    }
  }

  std::string response = "NUMLOGINS " + get_ups_name() + " " + std::to_string(num_logins) + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_get_upsdesc(NutClient &client, const std::string &args) {
  if (args != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  std::string response = "UPSDESC " + get_ups_name() + " \"" + get_ups_description() + "\"\n";
  send_response(client, response);
}

void NutServerComponent::handle_get_desc(NutClient &client, const std::string &args) {
  auto parts = split_args(args);
  if (parts.size() != 2) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  if (parts[0] != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  // Return a generic description for all supported variables
  std::string response = "DESC " + get_ups_name() + " " + parts[1] + " \"" + parts[1] + "\"\n";
  send_response(client, response);
}

void NutServerComponent::handle_get_type(NutClient &client, const std::string &args) {
  auto parts = split_args(args);
  if (parts.size() != 2) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  if (parts[0] != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  // Check if the variable exists
  std::string value = get_ups_var(parts[1]);
  if (value.empty()) {
    send_error(client, "VAR-NOT-SUPPORTED");
    return;
  }

  // Determine type based on variable name
  // RW variables (writable)
  std::string var_type = "RO";
  if (parts[1] == "ups.delay.shutdown") {
    var_type = "RW STRING";
  }

  std::string response = "TYPE " + get_ups_name() + " " + parts[1] + " " + var_type + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_list_cmd(NutClient &client, const std::string &args) {
  if (args != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  std::string response = "BEGIN LIST CMD " + get_ups_name() + "\n";

  auto commands = get_available_commands();
  for (const auto &cmd : commands) {
    response += "CMD " + get_ups_name() + " " + cmd + "\n";
  }

  response += "END LIST CMD " + get_ups_name() + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_list_clients(NutClient &client) {
  std::string response = "BEGIN LIST CLIENT\n";

  uint32_t now = millis();
  // Note: clients_mutex_ is already held by the calling context (server loop)

  for (size_t i = 0; i < clients_.size(); ++i) {
    const auto &c = clients_[i];
    if (c.is_active()) {
      // Format: CLIENT <ip> <connected_time> <status>
      std::string status = c.is_authenticated() ? "authenticated" : "connected";
      uint32_t connected_time = (now - c.connect_time) / 1000; // seconds

      response += "CLIENT " + c.remote_ip + " " + std::to_string(connected_time) + " " + status + "\n";
    }
  }

  response += "END LIST CLIENT\n";
  send_response(client, response);
}

void NutServerComponent::handle_instcmd(NutClient &client, const std::string &args) {
  ESP_LOGD(TAG, "INSTCMD received with args: '%s'", args.c_str());

  auto parts = split_args(args);
  ESP_LOGD(TAG, "INSTCMD parsed into %zu parts", parts.size());
  for (size_t i = 0; i < parts.size(); ++i) {
    ESP_LOGD(TAG, "  Part %zu: '%s'", i, parts[i].c_str());
  }

  if (parts.size() != 2) {
    ESP_LOGW(TAG, "INSTCMD invalid argument count: %zu (expected 2)", parts.size());
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  ESP_LOGD(TAG, "UPS name comparison: received='%s', expected='%s'", parts[0].c_str(), get_ups_name().c_str());
  if (parts[0] != get_ups_name()) {
    ESP_LOGW(TAG, "INSTCMD unknown UPS: '%s'", parts[0].c_str());
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  ESP_LOGD(TAG, "Executing command: '%s'", parts[1].c_str());
  bool cmd_result = execute_command(parts[1]);
  ESP_LOGD(TAG, "Command execution result: %s", cmd_result ? "SUCCESS" : "FAILED");

  if (cmd_result) {
    send_response(client, "OK\n");
  } else {
    ESP_LOGW(TAG, "Command failed or not supported: %s", parts[1].c_str());
    send_error(client, "CMD-NOT-SUPPORTED");
  }
}

void NutServerComponent::handle_version(NutClient &client) {
  std::string response = "VERSION \"" + std::string(NUT_VERSION) + "\"\n";
  send_response(client, response);
}

void NutServerComponent::handle_netver(NutClient &client) {
  // NETVER returns the network protocol version only, no prefix
  // NUT network protocol version 1.3 is current standard
  send_response(client, "1.3\n");
}

void NutServerComponent::handle_help(NutClient &client) {
  std::string response = "Commands: HELP VERSION NETVER STARTTLS USERNAME PASSWORD LOGIN LOGOUT LIST GET SET INSTCMD FSD UPSDVER\n";
  send_response(client, response);
}

void NutServerComponent::handle_upsdver(NutClient &client) {
  std::string response = std::string(UPSD_VERSION) + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_starttls(NutClient &client) {
  // STARTTLS is not supported (we don't have TLS/SSL).
  // Most clients (Synology, upsmon) send this on every connection and
  // gracefully fall back to plaintext.
  send_response(client, "ERR FEATURE-NOT-SUPPORTED\n");
}

void NutServerComponent::handle_username(NutClient &client, const std::string &args) {
  if (args.empty()) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  client.temp_username = args;
  ESP_LOGD(TAG, "Received username: %s", args.c_str());
  send_response(client, "OK\n");
}

void NutServerComponent::handle_password(NutClient &client, const std::string &args) {
  if (args.empty()) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  client.temp_password = args;
  ESP_LOGD(TAG, "Received password (authentication attempt)");

  // Attempt authentication with stored credentials
  if (authenticate(client.temp_username, client.temp_password)) {
    client.state = ClientState::AUTHENTICATED;
    client.username = client.temp_username;
    client.login_attempts = 0;
    ESP_LOGI(TAG, "Client %s authenticated successfully as %s", client.remote_ip.c_str(), client.username.c_str());
    send_response(client, "OK\n");
  } else {
    client.login_attempts++;
    if (client.login_attempts >= MAX_LOGIN_ATTEMPTS) {
      ESP_LOGW(TAG, "Max login attempts exceeded, disconnecting client");
      disconnect_client(client);
    } else {
      send_error(client, "ACCESS-DENIED");
    }
  }

  // Clear temporary credentials
  client.temp_username.clear();
  client.temp_password.clear();
}

void NutServerComponent::handle_fsd(NutClient &client, const std::string &args) {
  // FSD (Forced Shutdown) - this is a critical command
  // For now, just acknowledge but don't actually shutdown
  ESP_LOGW(TAG, "FSD (Forced Shutdown) command received from client");
  send_response(client, "OK FSD-SET\n");
}

void NutServerComponent::handle_set_var(NutClient &client, const std::string &args) {
  // SET VAR is not supported in this implementation
  send_error(client, "CMD-NOT-SUPPORTED");
}

void NutServerComponent::handle_list_rwvar(NutClient &client, const std::string &args) {
  // No read-write variables supported
  if (args != get_ups_name()) {
    send_error(client, "UNKNOWN-UPS");
    return;
  }

  std::string response = "BEGIN LIST RW " + get_ups_name() + "\n";
  response += "END LIST RW " + get_ups_name() + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_list_enum(NutClient &client, const std::string &args) {
  // No enum variables supported
  auto parts = split_args(args);
  if (parts.size() != 2 || parts[0] != get_ups_name()) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  std::string response = "BEGIN LIST ENUM " + get_ups_name() + " " + parts[1] + "\n";
  response += "END LIST ENUM " + get_ups_name() + " " + parts[1] + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_list_range(NutClient &client, const std::string &args) {
  // No range variables supported
  auto parts = split_args(args);
  if (parts.size() != 2 || parts[0] != get_ups_name()) {
    send_error(client, "INVALID-ARGUMENT");
    return;
  }

  std::string response = "BEGIN LIST RANGE " + get_ups_name() + " " + parts[1] + "\n";
  response += "END LIST RANGE " + get_ups_name() + " " + parts[1] + "\n";
  send_response(client, response);
}

void NutServerComponent::handle_legacy_list_vars(NutClient &client, const std::string &ups_name) {
  // Legacy format for upsc -l: return simple variable names without quotes
  if (!has_ups_data()) {
    send_error(client, "DATA-STALE");
    return;
  }

  std::string response = "";

  // Simple list format for legacy upsc -l support
  response += "ups.mfr\n";
  response += "ups.model\n";
  response += "battery.charge\n";
  response += "input.voltage\n";
  response += "output.voltage\n";
  response += "ups.load\n";
  response += "battery.runtime\n";
  response += "ups.status\n";

  send_response(client, response);
}

bool NutServerComponent::send_response(NutClient &client, const std::string &response) {
#ifdef USE_ESP32
  // Count lines for logging
  size_t line_count = 0;
  for (char c : response) {
    if (c == '\n') line_count++;
  }

  if (line_count <= 1 && response.length() <= 200) {
    // Single-line response: log inline at DEBUG (strip trailing newline)
    std::string trimmed = response;
    while (!trimmed.empty() && (trimmed.back() == '\n' || trimmed.back() == '\r'))
      trimmed.pop_back();
    ESP_LOGD(TAG, "  -> %s", trimmed.c_str());
  } else {
    // Multi-line response: compact summary at DEBUG, per-line at VERBOSE
    ESP_LOGD(TAG, "  -> [%zu bytes, %zu lines]", response.length(), line_count);
    // Per-line content only at VERBOSE to avoid blocking the server task
    size_t pos = 0;
    while (pos < response.length()) {
      size_t nl = response.find('\n', pos);
      if (nl == std::string::npos) nl = response.length();
      std::string line = response.substr(pos, nl - pos);
      if (!line.empty()) {
        ESP_LOGV(TAG, "  -> %s", line.c_str());
      }
      pos = nl + 1;
    }
  }

  size_t total_sent = 0;
  size_t remaining = response.length();
  const char* data = response.c_str();
  uint32_t send_start = millis();

  while (remaining > 0) {
    // Check send timeout to prevent blocking the server task
    if ((millis() - send_start) > SEND_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Send timeout after %lums (sent %zu/%zu bytes) to %s, disconnecting",
               (unsigned long)(millis() - send_start), total_sent, response.length(),
               client.remote_ip.c_str());
      disconnect_client(client);
      return false;
    }

    int bytes_sent = send(client.socket_fd, data + total_sent, remaining, 0);
    if (bytes_sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Socket buffer full, yield briefly and retry (with timeout protection above)
        delay(1);
        continue;
      }
      if (errno == ECONNRESET || errno == EPIPE || errno == ENOTCONN) {
        ESP_LOGD(TAG, "Client %s connection reset (error %d)", client.remote_ip.c_str(), errno);
      } else {
        ESP_LOGW(TAG, "Send error: %d (sent %zu/%zu bytes) to %s",
                 errno, total_sent, response.length(), client.remote_ip.c_str());
      }
      disconnect_client(client);
      return false;
    }
    total_sent += bytes_sent;
    remaining -= bytes_sent;
  }
  return true;
#else
  return false;
#endif
}

bool NutServerComponent::send_error(NutClient &client, const std::string &error) {
  std::string response = "ERR " + error + "\n";
  return send_response(client, response);
}

bool NutServerComponent::authenticate(const std::string &username, const std::string &password) {
  if (!auth_enabled()) {
    // No authentication required
    return true;
  }
  for (const auto &user : users_) {
    if (username == user.username && password == user.password) {
      return true;
    }
  }
  return false;
}

std::string NutServerComponent::get_ups_var(const std::string &var_name) {
  if (!has_ups_data() || !ups_hid_) {
    return "";
  }

  auto snapshot = ups_hid_->get_ups_data();
  return resolve_ups_var(var_name, snapshot);
}

std::string NutServerComponent::resolve_ups_var(const std::string &var_name,
                                                const ups_hid::UpsData &data) {
  // Manufacturer / model
  std::string mfr = data.device.manufacturer.empty() ? "Unknown" : data.device.manufacturer;
  std::string model = data.device.model.empty() ? "Unknown UPS" : data.device.model;

  if (var_name == "ups.mfr") return mfr;
  if (var_name == "ups.model") return model;
  if (var_name == "device.mfr") return mfr;
  if (var_name == "device.model") return model;
  if (var_name == "device.type") return "ups";

  // Driver identification
  if (var_name == "driver.name") return "usbhid-ups";
  if (var_name == "driver.version") return NUT_VERSION;
  if (var_name == "driver.version.internal") return "ESPHome UPS HID";

  // Device information
  if (var_name == "ups.serial" && !data.device.serial_number.empty())
    return data.device.serial_number;
  if (var_name == "device.serial" && !data.device.serial_number.empty())
    return data.device.serial_number;
  if (var_name == "ups.firmware" && !data.device.firmware_version.empty())
    return data.device.firmware_version;

  // Battery variables
  if (var_name == "battery.charge") {
    float level = data.battery.is_valid() ? data.battery.level : NAN;
    if (!std::isnan(level) && level >= 0) return std::to_string(static_cast<int>(level));
  }
  if (var_name == "battery.voltage" && !std::isnan(data.battery.voltage))
    return format_nut_value(std::to_string(data.battery.voltage));
  if (var_name == "battery.voltage.nominal" && !std::isnan(data.battery.voltage_nominal))
    return format_nut_value(std::to_string(data.battery.voltage_nominal));
  if (var_name == "battery.runtime") {
    float rt = data.battery.runtime_minutes;
    if (!std::isnan(rt) && rt > 0) return std::to_string(static_cast<int>(rt * 60));
  }
  if (var_name == "battery.type" && !data.battery.type.empty())
    return data.battery.type;
  if (var_name == "battery.mfr.date" && !data.battery.mfr_date.empty())
    return data.battery.mfr_date;
  if (var_name == "battery.voltage.low" && !std::isnan(data.battery.config_voltage))
    return format_nut_value(std::to_string(data.battery.config_voltage));
  if (var_name == "battery.capacity" && !std::isnan(data.battery.design_capacity))
    return format_nut_value(std::to_string(data.battery.design_capacity));
  if (var_name == "battery.charge.low" && !std::isnan(data.battery.charge_low))
    return std::to_string(static_cast<int>(data.battery.charge_low));
  if (var_name == "battery.charge.warning" && !std::isnan(data.battery.charge_warning))
    return std::to_string(static_cast<int>(data.battery.charge_warning));

  // Input power variables
  if (var_name == "input.voltage") {
    float v = data.power.input_voltage;
    if (!std::isnan(v) && v > 0) return format_nut_value(std::to_string(v));
  }
  if (var_name == "input.voltage.nominal" && !std::isnan(data.power.input_voltage_nominal))
    return format_nut_value(std::to_string(data.power.input_voltage_nominal));
  if (var_name == "input.frequency" && !std::isnan(data.power.frequency))
    return format_nut_value(std::to_string(data.power.frequency));
  if (var_name == "input.transfer.low" && !std::isnan(data.power.input_transfer_low))
    return format_nut_value(std::to_string(data.power.input_transfer_low));
  if (var_name == "input.transfer.high" && !std::isnan(data.power.input_transfer_high))
    return format_nut_value(std::to_string(data.power.input_transfer_high));

  // Output power variables
  if (var_name == "output.voltage") {
    float v = data.power.output_voltage;
    if (!std::isnan(v) && v > 0) return format_nut_value(std::to_string(v));
  }
  if (var_name == "output.voltage.nominal" && !std::isnan(data.power.output_voltage_nominal))
    return format_nut_value(std::to_string(data.power.output_voltage_nominal));
  if (var_name == "output.current" && !std::isnan(data.power.output_current))
    return format_nut_value(std::to_string(data.power.output_current));
  if (var_name == "output.frequency" && !std::isnan(data.power.output_frequency))
    return format_nut_value(std::to_string(data.power.output_frequency));
  if (var_name == "output.frequency.nominal" && !std::isnan(data.power.input_voltage_nominal)) {
    if (!std::isnan(data.power.frequency))
      return std::to_string(static_cast<int>(std::round(data.power.frequency)));
    if (!std::isnan(data.power.input_voltage_nominal))
      return data.power.input_voltage_nominal <= 130.0f ? "60" : "50";
  }

  // Load and power variables
  if (var_name == "ups.load") {
    float lp = data.power.load_percent;
    if (!std::isnan(lp) && lp >= 0) return std::to_string(static_cast<int>(lp));
  }
  if (var_name == "ups.realpower" && !std::isnan(data.power.active_power))
    return std::to_string(static_cast<int>(data.power.active_power));
  if (var_name == "ups.realpower.nominal" && !std::isnan(data.power.realpower_nominal))
    return std::to_string(static_cast<int>(data.power.realpower_nominal));
  if (var_name == "ups.power.nominal" && !std::isnan(data.power.apparent_power_nominal))
    return std::to_string(static_cast<int>(data.power.apparent_power_nominal));

  // Beeper status
  if (var_name == "ups.beeper.status" && !data.config.beeper_status.empty())
    return data.config.beeper_status;

  // Delay configuration
  if (var_name == "ups.delay.shutdown" && data.config.delay_shutdown >= 0)
    return std::to_string(data.config.delay_shutdown);

  // Timer values (-1 means inactive/not counting down)
  if (var_name == "ups.timer.shutdown")
    return std::to_string(data.test.timer_shutdown >= 0 ? data.test.timer_shutdown : -1);
  if (var_name == "ups.timer.reboot")
    return std::to_string(data.test.timer_reboot >= 0 ? data.test.timer_reboot : -1);

  // Test result
  if (var_name == "ups.test.result" && !data.test.ups_test_result.empty())
    return data.test.ups_test_result;

  // UPS status (built from the snapshot, no extra mutex acquisitions)
  if (var_name == "ups.status")
    return get_ups_status(&data);

  // Debug diagnostics
  if (ups_hid_) {
    if (var_name == "ups.debug.read.status") {
      uint32_t age_ms = ups_hid_->get_data_age_ms();
      char buf[128];
      snprintf(buf, sizeof(buf), "proto=%s stale=%u/%u age=%ums",
               data.power.status.empty() ? "(empty)" : data.power.status.c_str(),
               data.power.status_stale_cycles,
               data.power.MAX_STALE_CYCLES,
               age_ms);
      return std::string(buf);
    }

    if (var_name == "ups.debug.reset.reason") {
      std::string reason = ups_hid_->get_reset_reason();
      return reason.empty() ? "Unknown" : reason;
    }

    const auto &log = ups_hid_->get_event_log();

    if (var_name == "ups.debug.event.count")
      return std::to_string(log.size());

    const std::string event_prefix = "ups.debug.event.";
    if (var_name.substr(0, event_prefix.size()) == event_prefix) {
      std::string idx_str = var_name.substr(event_prefix.size());
      if (idx_str != "count") {
        int idx = atoi(idx_str.c_str());
        if (idx >= 0) {
          std::string event = log.get_event(static_cast<size_t>(idx));
          if (!event.empty()) return event;
        }
      }
    }
  }

  return "";
}

std::string NutServerComponent::get_ups_name() {
  // Return a consistent UPS name
  // This will be set during component configuration
  return ups_name_.empty() ? "ups" : ups_name_;
}

std::string NutServerComponent::get_ups_description() {
  if (!has_ups_data()) {
    return "ESPHome UPS";
  }

  std::string manufacturer = get_ups_manufacturer();
  std::string model = get_ups_model();

  std::string desc = manufacturer;
  if (!desc.empty() && !model.empty()) {
    desc += " " + model;
  } else if (desc.empty()) {
    desc = "ESPHome UPS";
  }
  return desc;
}

std::vector<std::string> NutServerComponent::get_available_commands() {
  std::vector<std::string> commands;

  if (ups_hid_ && ups_hid_->is_connected()) {
    // Standard UPS HID commands that are always available
    commands.push_back("beeper.enable");
    commands.push_back("beeper.disable");
    commands.push_back("beeper.mute");
    commands.push_back("beeper.test");
    commands.push_back("test.battery.start.quick");
    commands.push_back("test.battery.start.deep");
    commands.push_back("test.battery.stop");
    // Standard NUT command names for panel/UPS tests
    commands.push_back("test.panel.start");
    commands.push_back("test.panel.stop");
    // Keep legacy names for compatibility
    commands.push_back("test.ups.start");
    commands.push_back("test.ups.stop");
  }

  return commands;
}

bool NutServerComponent::execute_command(const std::string &command) {
  if (!ups_hid_) {
    return false;
  }

  // Map NUT commands to specific UPS HID methods
  if (command == "beeper.enable") {
    return ups_hid_->beeper_enable();
  } else if (command == "beeper.disable") {
    return ups_hid_->beeper_disable();
  } else if (command == "beeper.mute") {
    return ups_hid_->beeper_mute();
  } else if (command == "beeper.test") {
    return ups_hid_->beeper_test();
  } else if (command == "test.battery.start.quick") {
    return ups_hid_->start_battery_test_quick();
  } else if (command == "test.battery.start.deep") {
    return ups_hid_->start_battery_test_deep();
  } else if (command == "test.battery.stop") {
    return ups_hid_->stop_battery_test();
  } else if (command == "test.panel.start" || command == "test.ups.start") {
    return ups_hid_->start_ups_test();
  } else if (command == "test.panel.stop" || command == "test.ups.stop") {
    return ups_hid_->stop_ups_test();
  } else if (command == "debug.event.clear") {
    ups_hid_->get_event_log_mut().clear();
    return true;
  }

  return false;
}

std::string NutServerComponent::format_nut_value(const std::string &value) {
  // Format floating point values to 1 decimal place
  // Simple approach without exceptions
  char* endptr;
  float f = std::strtof(value.c_str(), &endptr);
  if (endptr != value.c_str() && *endptr == '\0') {
    // Valid float conversion
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << f;
    return oss.str();
  }
  return value;
}

std::vector<std::string> NutServerComponent::split_args(const std::string &args) {
  std::vector<std::string> result;
  std::istringstream iss(args);
  std::string token;

  while (iss >> token) {
    if (token.front() == '"') {
      std::string quoted = token.substr(1);
      if (quoted.empty() || quoted.back() != '"') {
        std::string rest;
        std::getline(iss, rest, '"');
        quoted += rest;
      } else {
        quoted.pop_back();
      }
      result.push_back(quoted);
    } else {
      result.push_back(token);
    }
  }

  return result;
}

bool NutServerComponent::has_ups_data() const {
  return ups_hid_ && ups_hid_->is_connected();
}

std::string NutServerComponent::get_ups_status(const ups_hid::UpsData *snapshot) const {
  if (!ups_hid_ || !ups_hid_->is_connected()) {
    return "";
  }

  // Use provided snapshot or fetch a fresh one.
  ups_hid::UpsData local;
  if (!snapshot) {
    local = ups_hid_->get_ups_data();
    snapshot = &local;
  }

  const auto &d = *snapshot;

  // Determine online/on-battery from the data directly (avoids data_mutex_).
  const auto &s = d.power.status;
  bool on_battery = (s == "On Battery");
  bool online = !on_battery && (!s.empty() && s != "Unknown");
  if (!online && !on_battery)
    online = d.power.input_voltage_valid();

  std::string result;
  auto append = [&](const char *flag) {
    if (!result.empty()) result += ' ';
    result += flag;
  };

  if (online)      append("OL");
  else if (on_battery) append("OB");

  if (d.battery.is_low())  append("LB");

  bool charging = online && d.battery.is_valid() &&
                  !std::isnan(d.battery.level) && d.battery.level < 100.0f;
  if (charging) append("CHRG");

  if (d.power.boost_active)      append("BOOST");
  if (d.power.buck_active)       append("TRIM");
  if (d.power.over_temperature)  append("OVER");
  if (d.power.shutdown_imminent) append("FSD");
  if (d.power.awaiting_power)    append("OFF");
  if (d.battery.needs_replacement) append("RB");

  bool fault = d.power.is_input_out_of_range() ||
               (!d.power.is_valid() && !d.battery.is_valid());
  if (fault) append("ALARM");

  return result;
}

std::string NutServerComponent::get_ups_manufacturer(const ups_hid::UpsData *snapshot) const {
  if (ups_hid_) {
    if (snapshot) {
      if (!snapshot->device.manufacturer.empty()) return snapshot->device.manufacturer;
    } else {
      auto d = ups_hid_->get_ups_data();
      if (!d.device.manufacturer.empty()) return d.device.manufacturer;
    }
  }
  return "Unknown";
}

std::string NutServerComponent::get_ups_model(const ups_hid::UpsData *snapshot) const {
  if (ups_hid_) {
    if (snapshot) {
      if (!snapshot->device.model.empty()) return snapshot->device.model;
    } else {
      auto d = ups_hid_->get_ups_data();
      if (!d.device.model.empty()) return d.device.model;
    }
  }
  return "Unknown UPS";
}

}  // namespace nut_server
}  // namespace esphome
