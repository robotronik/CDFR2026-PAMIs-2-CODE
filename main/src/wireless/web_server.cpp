#include "wireless/web_server.h"
#include <esp_http_server.h>
#include <esp_log.h>
#include <string.h>

static const char *LOGGER_TAG = "WEB_LOG";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static httpd_handle_t server = NULL;
static int ws_fd = -1; 
static vprintf_like_t default_vprintf = NULL;

int web_log_vprintf(const char *fmt, va_list args) {
    char temp_buf[256];
    int len = vsnprintf(temp_buf, sizeof(temp_buf), fmt, args);

    if (len > 0 && ws_fd != -1) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)temp_buf;
        ws_pkt.len = len;
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        if (httpd_ws_send_frame_async(server, ws_fd, &ws_pkt) != ESP_OK) { 
            ws_fd = -1;
        }
    }

    return default_vprintf(fmt, args);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGD(LOGGER_TAG, "Live Console: Browser connected (FD: %d)", ws_fd);
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req) {
    const size_t index_html_size = (index_html_end - index_html_start);
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_size);
}

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.core_id = 0;

    ESP_LOGD(LOGGER_TAG, "Starting Real-time Log Server...");

    if (httpd_start(&server, &config) == ESP_OK) { 
        httpd_uri_t root = {};
        root.uri = "/";
        root.method = HTTP_GET;
        root.handler = root_get_handler;
        root.user_ctx = NULL;
        httpd_register_uri_handler(server, &root);
 
        httpd_uri_t ws = {};
        ws.uri          = "/ws";
        ws.method       = HTTP_GET;
        ws.handler      = ws_handler;
        ws.user_ctx     = NULL;
        ws.is_websocket = true;
        httpd_register_uri_handler(server, &ws);
 
        default_vprintf = esp_log_set_vprintf(web_log_vprintf);
        
        ESP_LOGD(LOGGER_TAG, "Server running! Open your browser to view logs.");
    } else {
        ESP_LOGE(LOGGER_TAG, "Failed to start web server!");
    }
}
