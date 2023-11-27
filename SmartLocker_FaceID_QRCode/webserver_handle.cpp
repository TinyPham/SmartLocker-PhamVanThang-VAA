#include "img_converters.h"
#include "fb_gfx.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "zip_interface.h"
#include "Arduino.h"
#include <EEPROM.h>

#include "fd_forward.h"
#include "fr_flash.h"
#include "fr_forward.h"

#include "quirc.h"
String getValue(String data, char separator, int index);
void handle_QR();

void write_eeprom_qr_locker();
void read_eeprom_qr_locker();
void delete_eeprom(int qrLockerNumber) ;

void write_eeprom_disable_locker();
void read_eeprom_disable_locker();
void delete_eeprom_disable_locker(int disableLockerNumber);

int8_t enroll_face_id_to_flash_extra(face_id_list *l,
              dl_matrix3du_t *aligned_face);
              
int8_t recognize_face_extra(face_id_list *l,
                        dl_matrix3du_t *algined_face);
void delete_face(int faceNumber); 
TaskHandle_t QRCodeReader_Task; 

struct QRCodeData
{
  bool valid;
  int8_t dataType;
  uint8_t payload[1024];
  int8_t payloadLen;
};

struct quirc *q = NULL;
uint8_t *image = NULL;  
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
struct QRCodeData qrCodeData;  
bool ws_run = false;
int8_t wsLive_val = 0;
int8_t last_wsLive_val;
byte get_wsLive_interval = 0;
bool get_wsLive_val = true;

unsigned long previousMillis = 0;

esp_partition_type_t flash_type = static_cast<esp_partition_type_t>(FR_FLASH_TYPE);
esp_partition_subtype_t flash_subtype = static_cast<esp_partition_subtype_t>(FR_FLASH_SUBTYPE);

#define ENROLL_CONFIRM_TIMES 2
#define FACE_ID_SAVE_NUMBER 4

String resultData = "";
extern String pickLocker;
extern String emergenUnlocker1;
extern String emergenUnlocker2;
extern String emergenUnlocker3;
extern String emergenUnlocker4;
String checkFace = "";
int8_t countCheck = 0;
String conditionReadQr = "0";
String QRCodeResultSend = "";
String disableLockerSend = "";

int index_name = 0;

extern String disableLocker1;
extern String disableLocker2;
extern String disableLocker3;
extern String disableLocker4;

extern String lockerStatus;

extern String regisQr_1;
extern String regisQr_2;
extern String regisQr_3;
extern String regisQr_4;

extern String QRCodeResult;
extern String conditionOpenCameraQR;
extern String controlLockerbyQr;

String recognize_face_matched_name[4] = {"LOCKER 1", "LOCKER 2", "LOCKER 3","LOCKER 4"};
#define FACE_COLOR_WHITE  0x00FFFFFF

#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
httpd_handle_t qr_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};

extern boolean matchFace1;
extern boolean matchFace2;
extern boolean matchFace3;
extern boolean matchFace4;

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){
        color = FACE_COLOR_RED;
    } else if(face_id > 0){
        color = FACE_COLOR_GREEN;
    }
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    for (i = 0; i < boxes->len; i++){
        // rectangle box
        x = (int)boxes->box[i].box_p[0];
        y = (int)boxes->box[i].box_p[1];
        w = (int)boxes->box[i].box_p[2] - x + 1;
        h = (int)boxes->box[i].box_p[3] - y + 1;
        fb_gfx_drawFastHLine(&fb, x, y, w, color);
        fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
        fb_gfx_drawFastVLine(&fb, x, y, h, color);
        fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
    }
}


static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    static dl_matrix3du_t *aligned_face = NULL;
    int matched_id = -1;
    const esp_partition_t *pt = esp_partition_find_first(flash_type, flash_subtype, FR_FLASH_PARTITION_NAME);
    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
      if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        
          if (is_enrolling == 1)
          {
            checkFace = "";
                if(pickLocker == "LOCKER_1")
                {
                  id_list.head = 0;
                  id_list.tail = 0;
                }
                if(pickLocker == "LOCKER_2")
                {
                  id_list.head = 1;
                  id_list.tail = 1;
                }
                if(pickLocker == "LOCKER_3")
                {
                  id_list.head = 2;
                  id_list.tail = 2;
                }
                if(pickLocker == "LOCKER_4")
                {
                  id_list.head = 3;
                  id_list.tail = 3;
                }
                int8_t left_sample_face = enroll_face_id_to_flash_extra(&id_list, aligned_face);
                if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                    Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
                }
                Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
                rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
                if (left_sample_face == 0){
                    is_enrolling = 0;
                    Serial.printf("Enrolled Face ID: %d\n", id_list.tail); 
                }
         }
        
          else 
          {
              Serial.print("pickLocker: ");
              Serial.println(pickLocker);
              matched_id = recognize_face_extra(&id_list, aligned_face);
              
              if (matched_id == 0) 
              {
                  Serial.printf("Match Face Name: %s\n", recognize_face_matched_name[0]);
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] %s", matched_id, recognize_face_matched_name[0]);
                  
                  countCheck++;
                  if(countCheck == 2)
                  {
                    lockerStatus =  matched_id;
                    matchFace1 = true;
                  }
                  if(checkFace == "CHECKFACE" && countCheck == 3)
                  {
                    id_list.head = 0;
                    delete_face_id_in_flash(&id_list);
                    id_list.count = 0;
                    id_list.head = 0;
                    Serial.print("countFace: ");
                    Serial.println(id_list.count);
                    disableLocker1 = "0";
                    Serial.println("Deleting Face: LOCKER 1");             
                    lockerStatus = -10;   
                    countCheck = 0;                
                  }                          
              } 
  
              else if (matched_id == 1) 
              {
                  Serial.printf("Match Face Name: %s\n", recognize_face_matched_name[1]);
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] %s", matched_id, recognize_face_matched_name[1]);
                  countCheck++;
                  if(countCheck == 2)
                  {
                    lockerStatus =  matched_id;
                    matchFace2 = true;
                  }
                  if(checkFace == "CHECKFACE" && countCheck == 3)
                  { 
                    id_list.head = 1;
                    delete_face_id_in_flash(&id_list);
                    id_list.count = 0;
                    id_list.head = 1;
                    Serial.print("countFace: ");
                    Serial.println(id_list.count);
                    disableLocker2 = "0";
                    Serial.println("Deleting Face: LOCKER 2");
                    lockerStatus = -11;   
                    countCheck = 0;                       
                  }                         
              } 
  
              else if (matched_id == 2) 
              {
                  Serial.printf("Match Face Name: %s\n", recognize_face_matched_name[2]);
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] %s", matched_id, recognize_face_matched_name[2]);
                  countCheck++;
                  if(countCheck == 2)
                  {
                    lockerStatus =  matched_id;
                    matchFace3 = true;
                  }
                  if(checkFace == "CHECKFACE" && countCheck == 3)
                  {
                    id_list.head = 2;
                    delete_face_id_in_flash(&id_list);
                    id_list.count = 0;
                    id_list.head = 2;
                    Serial.print("countFace: ");
                    Serial.println(id_list.count);
                    disableLocker3 = "0";
                    Serial.println("Deleting Face: LOCKER 3");
                    lockerStatus = -12;      
                    countCheck = 0;                     
                  }                         
              }
  
              else if (matched_id == 3) 
              {
                  Serial.printf("Match Face Name: %s\n", recognize_face_matched_name[3]);
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] %s", matched_id, recognize_face_matched_name[3]);
                  delay(200);
                  countCheck++;
                  if(countCheck == 2)
                  {
                    lockerStatus =  matched_id;
                    matchFace4 = true;
                  }
                  if(checkFace == "CHECKFACE" && countCheck == 3)
                  {
                    id_list.head = 3;
                    delete_face_id_in_flash(&id_list);
                    id_list.count = 0;
                    id_list.head = 3;
                    Serial.print("countFace: ");
                    Serial.println(id_list.count);
                    disableLocker4 = "0";
                    Serial.println("Deleting Face: LOCKER 4");
                    lockerStatus = -13;     
                    countCheck = 0;                       
                  }                                           
              }
              else 
              {
                  Serial.println("No Match Found");
                  rgb_print(image_matrix, FACE_COLOR_RED, "DENY FACE!");
                  countCheck = 0;
              } 
           }
     }             
    else 
    {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    bool detected = false;
    int face_id = 0;
    if(!detection_enabled || fb->width > 400){
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    out_buf = image_matrix->item;
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes){
        detected = true;
        if(recognition_enabled){
            face_id = run_face_recognition(image_matrix, net_boxes);
        }
        draw_face_boxes(image_matrix, net_boxes, face_id);
        dl_lib_free(net_boxes->score);
        dl_lib_free(net_boxes->box);
        dl_lib_free(net_boxes->landmark);
        dl_lib_free(net_boxes);
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    

    while(true){
        detected = false;
        face_id = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(!detection_enabled || fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(net_boxes){
                                detected = true;
                                if(recognition_enabled){
                                    face_id = run_face_recognition(image_matrix, net_boxes);
                                }
                                fr_recognize = esp_timer_get_time();
                                draw_face_boxes(image_matrix, net_boxes, face_id);
                                dl_lib_free(net_boxes->score);
                                dl_lib_free(net_boxes->box);
                                dl_lib_free(net_boxes->landmark);
                                dl_lib_free(net_boxes);
                            }
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time,
            (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
            (detected)?"DETECTED ":"", face_id
        );      
    }

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;
    if(!strcmp(variable, "framesize")) {
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    if(!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if(!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if(!strcmp(variable, "face_detect")) {
        detection_enabled = val;
        if(!detection_enabled) {
            recognition_enabled = 0;
        }
    }
    else if(!strcmp(variable, "face_enroll")) is_enrolling = val;
    else if(!strcmp(variable, "face_recognize")) {
        recognition_enabled = val;
        if(recognition_enabled){
            detection_enabled = val;
        }
    }
    else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p+=sprintf(p, "\"face_detect\":%u,", detection_enabled);
    p+=sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    p+=sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t * s = esp_camera_sensor_get();
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

static esp_err_t cmd_handler2(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
   
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int res = 0;

  Serial.print("Incoming command : ");
  Serial.println(variable);
  Serial.println();
  String getData = String(variable);
  resultData = getValue(getData, ',', 0);
  if (resultData == "A")
  {
    lockerStatus = getValue(getData, ',', 1);
    Serial.print("lockerStatus =   ");
    Serial.println(lockerStatus);
  }
  if (resultData == "B")
  {
    pickLocker = getValue(getData, ',', 1);
    Serial.print("pickLocker =   ");
    Serial.println(pickLocker);
  }
  if (resultData == "C")
  {
    checkFace = getValue(getData, ',', 1);
    if(checkFace == "CHECKFACE") countCheck = 0;
    Serial.print("checkFace =   ");
    Serial.println(checkFace);
  }
  if (resultData == "D")
  {
    countCheck = getValue(getData, ',', 1).toInt();
    Serial.print("countCheck =   ");
    Serial.println(countCheck);
    if(countCheck == 0) 
    {
      checkFace = "";
    }   
  }
  if (resultData == "E")
  {
    if(pickLocker == "LOCKER_1")
    {
      regisQr_1 = getValue(getData, ',', 1);
      Serial.print("regisQr_1 =   ");
      Serial.println(regisQr_1);
    }
    else if(pickLocker == "LOCKER_2")
    {
      regisQr_2 = getValue(getData, ',', 1);
      Serial.print("regisQr_2 =   ");
      Serial.println(regisQr_2);
    }
    else if(pickLocker == "LOCKER_3")
    {
      regisQr_3 = getValue(getData, ',', 1);
      Serial.print("regisQr_3 =   ");
      Serial.println(regisQr_3);
    }
    else if(pickLocker == "LOCKER_4")
    {
      regisQr_4 = getValue(getData, ',', 1);
      Serial.print("regisQr_4 =   ");
      Serial.println(regisQr_4);
    }
  }
  if (resultData == "F")
  {
    controlLockerbyQr = getValue(getData, ',', 1);
    Serial.print("controlLockerbyQr =   ");
    Serial.println(controlLockerbyQr);
    if(controlLockerbyQr == "1")
    {
      write_eeprom_qr_locker();
    }
  }
  if (resultData == "X")
  {
    emergenUnlocker1 = getValue(getData, ',', 1);
    if(emergenUnlocker1 == "1")
    {
      disableLocker1 = "0";
      QRCodeResult = "x@x";
      regisQr_1 = "";
      delete_eeprom(1);
      delete_eeprom_disable_locker(1);
    }
    Serial.print("emergenUnlocker1 =   ");
    Serial.println(emergenUnlocker1);
  }

  if (resultData == "Y")
  {
    emergenUnlocker2 = getValue(getData, ',', 1);
    if(emergenUnlocker2 == "1")
    {
      disableLocker2 = "0";
      QRCodeResult = "x@x";
      regisQr_2 = "";
      delete_eeprom(2);  
      delete_eeprom_disable_locker(2);
    }
    Serial.print("emergenUnlocker2 =   ");
    Serial.println(emergenUnlocker2);
  }

  if (resultData == "Z")
  {
    emergenUnlocker3 = getValue(getData, ',', 1);
    if(emergenUnlocker3 == "1")
    {
      disableLocker3 = "0";
      QRCodeResult = "x@x";
      regisQr_3 = "";
      delete_eeprom(3);  
      delete_eeprom_disable_locker(3);
    }
    Serial.print("emergenUnlocker3 =   ");
    Serial.println(emergenUnlocker3);
  }

  if (resultData == "T")
  {
    emergenUnlocker4 = getValue(getData, ',', 1);
    if(emergenUnlocker4 == "1")
    {
      disableLocker4 = "0";
      QRCodeResult = "x@x";
      regisQr_4 = "";
      delete_eeprom(4); 
      delete_eeprom_disable_locker(4);
    }
    Serial.print("emergenUnlocker4 =   ");
    Serial.println(emergenUnlocker4);
  }

  if (resultData == "K")
  {
    disableLocker1 = getValue(getData, ',', 1);
    Serial.print("disableLocker1 =   ");
    Serial.println(disableLocker1);
    write_eeprom_disable_locker();
  }

  if (resultData == "L")
  {
    disableLocker2 = getValue(getData, ',', 1);
    Serial.print("disableLocker2 =   ");
    Serial.println(disableLocker2);
    write_eeprom_disable_locker();
  }

  if (resultData == "M")
  {
    disableLocker3 = getValue(getData, ',', 1);
    Serial.print("disableLocker3 =   ");
    Serial.println(disableLocker3);
    write_eeprom_disable_locker();
  }

  if (resultData == "N")
  {
    disableLocker4 = getValue(getData, ',', 1);
    Serial.print("disableLocker4 =   ");
    Serial.println(disableLocker4);
    write_eeprom_disable_locker();
  }
}

static esp_err_t locker_handler(httpd_req_t *req){

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, lockerStatus.c_str(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t disable_locker_handler(httpd_req_t *req){
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if(disableLocker1 != "1" && disableLocker2 != "1" && disableLocker3 != "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12340000";
    }
    if(disableLocker1 == "1" && disableLocker2 != "1" && disableLocker3 != "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12341000";
    }
    if(disableLocker1 != "1" && disableLocker2 == "1" && disableLocker3 != "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12340100";
    }
    if(disableLocker1 != "1" && disableLocker2 != "1" && disableLocker3 == "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12340010";
    }
    if(disableLocker1 != "1" && disableLocker2 != "1" && disableLocker3 != "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12340001";
    }
    if(disableLocker1 == "1" && disableLocker2 != "1" && disableLocker3 != "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12341001";
    }
    if(disableLocker1 == "1" && disableLocker2 != "1" && disableLocker3 == "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12341010";
    }
    if(disableLocker1 == "1" && disableLocker2 == "1" && disableLocker3 != "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12341100";
    }
    if(disableLocker1 != "1" && disableLocker2 == "1" && disableLocker3 == "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12340110";
    }
    if(disableLocker1 != "1" && disableLocker2 == "1" && disableLocker3 != "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12340101";
    }
    if(disableLocker1 != "1" && disableLocker2 != "1" && disableLocker3 == "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12340011";
    }
    if(disableLocker1 == "1" && disableLocker2 == "1" && disableLocker3 == "1" && disableLocker4 != "1")
    {
      disableLockerSend = "12341110";
    }
    if(disableLocker1 == "1" && disableLocker2 == "1" && disableLocker3 != "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12341101";
    }
    if(disableLocker1 != "1" && disableLocker2 == "1" && disableLocker3 == "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12340111";
    }
    if(disableLocker1 == "1" && disableLocker2 != "1" && disableLocker3 == "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12341011";
    }
    if(disableLocker1 == "1" && disableLocker2 == "1" && disableLocker3 == "1" && disableLocker4 == "1")
    {
      disableLockerSend = "12341111";
    }
    return httpd_resp_send(req, disableLockerSend.c_str(), HTTPD_RESP_USE_STRLEN);
}

void dumpData(const struct quirc_data *data)
{
  Serial.printf("-Version: %d\n", data->version);
  Serial.printf("-ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("-Mask: %d\n", data->mask);
  Serial.printf("-Length: %d\n", data->payload_len);
  Serial.printf("-Payload: %s\n", data->payload);
  
  QRCodeResult = (const char *)data->payload;
}

static esp_err_t qr_reader_handler(httpd_req_t *req){
  ws_run = true;
  vTaskDelete(QRCodeReader_Task);
  Serial.print("stream_handler running on core ");
  Serial.println(xPortGetCoreID());

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  /* ---------------------------------------- Loop to show streaming video from ESP32 Cam camera and read QR Code. */
    while(true){
      ws_run = true;
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed (stream_handler)");
        res = ESP_FAIL;
      } else {
        q = quirc_new();
        if (q == NULL){
          Serial.print("can't create quirc object\r\n");  
          continue;
        }
        
        quirc_resize(q, fb->width, fb->height);
        image = quirc_begin(q, NULL, NULL);
        memcpy(image, fb->buf, fb->len);
        quirc_end(q);
        
        int count = quirc_count(q);
        if (count > 0) {
          quirc_extract(q, 0, &code);
          err = quirc_decode(&code, &data);
      
          if (err){
            QRCodeResult = "Decoding FAILED";
            Serial.println(QRCodeResult);
          } else {
            Serial.printf("Decoding successful:\n");
            dumpData(&data);
          } 
          Serial.println();
        } 
        
        image = NULL;  
        quirc_destroy(q);
        
        if(fb->width > 200){
          if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            esp_camera_fb_return(fb);
            fb = NULL;
            if(!jpeg_converted){
              Serial.println("JPEG compression failed");
              res = ESP_FAIL;
            }
          } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
          }
        }
      }
      if(res == ESP_OK){
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      if(fb){
        esp_camera_fb_return(fb);
        fb = NULL;
        _jpg_buf = NULL;
      } else if(_jpg_buf){
        free(_jpg_buf);
        _jpg_buf = NULL;
      }
      if(res != ESP_OK){
        break;
      }
      
      wsLive_val++;
      if (wsLive_val > 999) wsLive_val = 0;
    }
  /* ---------------------------------------- */
  return res;
}

static esp_err_t qrcoderslt_handler(httpd_req_t *req){
  if (QRCodeResult != "Decoding FAILED") QRCodeResultSend = QRCodeResult;
  httpd_resp_send(req, QRCodeResultSend.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void QRCodeReader( void * pvParameters ){
  Serial.print("QRCodeReader running on core ");
  Serial.println(xPortGetCoreID());

  while(!ws_run)
  {
      camera_fb_t * fb = NULL;
      q = quirc_new();

      if (q == NULL){
        Serial.print("can't create quirc object\r\n");  
        continue;
      }
  
        fb = esp_camera_fb_get();
        if (!fb)
        {
          //Serial.println("Camera capture failed (QRCodeReader())");
          continue;
        }
      
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);
      memcpy(image, fb->buf, fb->len);
      quirc_end(q);
      
      int count = quirc_count(q);
      if (count > 0) {
        //Serial.println(count);
        quirc_extract(q, 0, &code);
        err = quirc_decode(&code, &data);
    
        if (err){
          QRCodeResult = "Decoding FAILED";
          Serial.println(QRCodeResult);
        } else {
          Serial.printf("Decoding successful:\n");
          dumpData(&data);
        } 
        Serial.println();
      } 
      
      esp_camera_fb_return(fb);
      fb = NULL;
      image = NULL;  
      quirc_destroy(q);
  }
}

void createTaskQRCodeReader() {
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Task function. */
             "QRCodeReader_Task",   /* name of task. */
             10000,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             1,                     /* priority of the task */
             &QRCodeReader_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 0 */
}


void handle_QR()
{
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 200) {
      previousMillis = currentMillis;
  
      if (ws_run == true) {
        if (get_wsLive_val == true) {
          last_wsLive_val = wsLive_val;
          get_wsLive_val = false;
        }
     
        get_wsLive_interval++;
        if (get_wsLive_interval > 2) {
          get_wsLive_interval = 0;
          get_wsLive_val = true;
          if (wsLive_val == last_wsLive_val) {
            ws_run = false;
            last_wsLive_val = 0;
            createTaskQRCodeReader();
          }
        }
      }
    }
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t locker_uri = {
    .uri       = "/getlockerval",
    .method    = HTTP_GET,
    .handler   = locker_handler,
    .user_ctx  = NULL
  };

    httpd_uri_t disable_locker_uri = {
    .uri       = "/getdisablelockerval",
    .method    = HTTP_GET,
    .handler   = disable_locker_handler,
    .user_ctx  = NULL
  };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };


    httpd_uri_t cmd_uri2 = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler2,
    .user_ctx  = NULL
  };

  httpd_uri_t qrcoderslt_uri = {
    .uri       = "/getqrcodeval",
    .method    = HTTP_GET,
    .handler   = qrcoderslt_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t qr_reader_uri = {
    .uri       = "/qr_reader",
    .method    = HTTP_GET,
    .handler   = qr_reader_handler,
    .user_ctx  = NULL
  };

    ra_filter_init(&ra_filter, 20);
    
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    read_face_id_from_flash(&id_list);
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &locker_uri);
        httpd_register_uri_handler(camera_httpd, &disable_locker_uri);   
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri2);       
        httpd_register_uri_handler(camera_httpd, &qrcoderslt_uri);                    
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);   
        httpd_register_uri_handler(stream_httpd, &qr_reader_uri);    
    }
}

void write_eeprom_qr_locker()
{
    EEPROM.begin(512);
    for (int i = 250; i < 410; ++i)
    {
      EEPROM.write(i, 0);
    }
    for (int i = 0; i < regisQr_1.length(); i++)
    {
      EEPROM.write(250 + i, regisQr_1[i]);
    }
    for (int i = 0; i < regisQr_2.length(); i++)
    {
      EEPROM.write(290 + i, regisQr_2[i]);
    }
    for (int i = 0; i < regisQr_3.length(); i++)
    {
      EEPROM.write(330 + i, regisQr_3[i]);
    }
    for (int i = 0; i < regisQr_4.length(); i++)
    {
      EEPROM.write(370 + i, regisQr_4[i]);
    }
    EEPROM.commit(); 
}

void read_eeprom_qr_locker()
{
  EEPROM.begin(512);
  for (int i = 250; i < 290; ++i)
  {
    regisQr_1 += char(EEPROM.read(i));
  }
  regisQr_1.remove(regisQr_1.indexOf(0));
  for (int i = 290; i < 330; ++i)
  {
    regisQr_2 += char(EEPROM.read(i));
  }
  regisQr_2.remove(regisQr_2.indexOf(0));
  for (int i = 330; i < 370; ++i)
  {
    regisQr_3 += char(EEPROM.read(i));
  }
  regisQr_3.remove(regisQr_3.indexOf(0));
  for (int i = 370; i < 410; ++i)
  {
    regisQr_4 += char(EEPROM.read(i));
  }
  regisQr_4.remove(regisQr_4.indexOf(0));
  delay(1000);
  Serial.print("regisQr_1: ");
  Serial.println(regisQr_1);
  Serial.print("regisQr_2: ");
  Serial.println(regisQr_2);
  Serial.print("regisQr_3: ");
  Serial.println(regisQr_3);
  Serial.print("regisQr_4: ");
  Serial.println(regisQr_4);
}

void delete_eeprom(int qrLockerNumber) 
{
  int i_start = 210 + qrLockerNumber*40;
  int i_end = 210 + ((qrLockerNumber+1)*40);
  EEPROM.begin(512);
  for (int i = i_start; i < i_end; i++) 
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
}


void write_eeprom_disable_locker()
{
    EEPROM.begin(512);
    for (int i = 240; i < 250; ++i)
    {
      EEPROM.write(i, 0);
    }
    for (int i = 0; i < disableLocker1.length(); i++)
    {
      EEPROM.write(240 + i, disableLocker1[i]);
    }
    Serial.println("write eeprom disable locker 1 DONE............");
    for (int i = 0; i < disableLocker2.length(); i++)
    {
      EEPROM.write(242 + i, disableLocker2[i]);
    }
    Serial.println("write eeprom disable locker 2 DONE............");
    for (int i = 0; i < disableLocker3.length(); i++)
    {
      EEPROM.write(244 + i, disableLocker3[i]);
    }
    Serial.println("write eeprom disable locker 3 DONE............");
    for (int i = 0; i < disableLocker4.length(); i++)
    {
      EEPROM.write(246 + i, disableLocker4[i]);
    }
    Serial.println("write eeprom disable locker 4 DONE............");
    EEPROM.commit(); 
}

void read_eeprom_disable_locker()
{
  EEPROM.begin(512);
  for (int i = 240; i < 242; ++i)
  {
    disableLocker1 += char(EEPROM.read(i));
  }
  disableLocker1.remove(disableLocker1.indexOf(0));
  for (int i = 242; i < 244; ++i)
  {
    disableLocker2 += char(EEPROM.read(i));
  }
  disableLocker2.remove(disableLocker2.indexOf(0));
  for (int i = 244; i < 246; ++i)
  {
    disableLocker3 += char(EEPROM.read(i));
  }
  disableLocker3.remove(disableLocker3.indexOf(0));
  for (int i = 246; i < 248; ++i)
  {
    disableLocker4 += char(EEPROM.read(i));
  }
  disableLocker4.remove(disableLocker4.indexOf(0));
  delay(1000);
  Serial.print("disableLocker1: ");
  Serial.println(disableLocker1);
  Serial.print("disableLocker2: ");
  Serial.println(disableLocker2);
  Serial.print("disableLocker3: ");
  Serial.println(disableLocker3);
  Serial.print("disableLocker4: ");
  Serial.println(disableLocker4);
}

void delete_eeprom_disable_locker(int disableLockerNumber) 
{
  int i_start = 238 + disableLockerNumber*2;
  int i_end = 238 + ((disableLockerNumber+1)*2);
  EEPROM.begin(512);
  for (int i = i_start; i < i_end; i++) 
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
}

void delete_face(int faceNumber) 
{
  int i_start = 4096 + faceNumber*2048;
  int i_end = 4096 + ((faceNumber+1)*2048);
  EEPROM.begin(512);
  for (int i = i_start; i < i_end; i++) 
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
}

fptp_t cos_distance(dl_matrix3d_t *id_1,
                    dl_matrix3d_t *id_2)
{
    //assert(id_1->c == id_2->c);
    uint16_t c = id_1->c;
    fptp_t l2_norm_1 = 0;
    fptp_t l2_norm_2 = 0;
    fptp_t dist = 0;
    for (int i = 0; i < c; i++)
    {
        l2_norm_1 += ((id_1->item[i]) * (id_1->item[i]));
        l2_norm_2 += ((id_2->item[i]) * (id_2->item[i]));
    }
    l2_norm_1 = sqrt(l2_norm_1);
    l2_norm_2 = sqrt(l2_norm_2);
    for (uint16_t i = 0; i < c; i++)
    {
        dist += ((id_1->item[i]) * (id_2->item[i]) / (l2_norm_1 * l2_norm_2));
    }
    return dist;
}
int8_t recognize_face_extra(face_id_list *l,
                        dl_matrix3du_t *algined_face)
{
    fptp_t similarity = 0;
    fptp_t max_similarity = -1;
    int8_t matched_id = -1;
    static uint8_t head;
    dl_matrix3d_t *face_id = NULL;

    face_id = get_face_id(algined_face);
    if(pickLocker == "LOCKER_1" || pickLocker == "LOCKER_2" || pickLocker == "LOCKER_3" || pickLocker == "LOCKER_4")
    {
      for(int i = 0; i < l->count; i++)
      {
          Serial.println("recognize face at LOCKER_1_2_3_4: ");
          uint8_t head = l->count - 1;
          Serial.println("enterSOLVE");
          Serial.print("head enterSOLVE: ");
          Serial.println(head);
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              Serial.println("enterCOMPARE");
              max_similarity = similarity;
              matched_id = head;
          }
      }
    }

    if(pickLocker == "LOCKER")
    {
      if (disableLockerSend == "12341000") 
      {
        Serial.println("max_similarity at 1000: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340100") 
      {
        Serial.println("max_similarity at 0100: ");
          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340010") 
      {
        Serial.println("max_similarity at 0010: ");
          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340001") 
      {
          Serial.println("max_similarity at 0001: ");
          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341001") 
      {
        Serial.println("max_similarity at 1001: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341010") 
      {
        Serial.println("max_similarity at 1010: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341100") 
      {
        Serial.println("max_similarity at 1100: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340110") 
      {
        Serial.println("max_similarity at 0110: ");
          
          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340101") 
      {
        Serial.println("max_similarity at 0101: ");
          head = 1;
          Serial.print("max_similarity at 0101: ");
          Serial.println(max_similarity);
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }


      if (disableLockerSend == "12340011") 
      {
        Serial.println("max_similarity at 0011: ");
          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341110") 
      {
        Serial.println("max_similarity at 1110: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);

          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341101") 
      {
        Serial.println("max_similarity at 1101: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12340111") 
      {
        Serial.println("max_similarity at 0111: ");
          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }

      if (disableLockerSend == "12341011") 
      {
        Serial.println("max_similarity at 1011: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
        }

      if (disableLockerSend == "12341111") 
      {
        Serial.println("max_similarity at 1111: ");
          head = 0;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 1;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 2;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }

          head = 3;
          similarity = cos_distance(l->id_list[head], face_id);
  
          if (similarity > max_similarity)
          {
              max_similarity = similarity;
              matched_id = head;
          }
      }
    }

    if (max_similarity < FACE_REC_THRESHOLD)
    {
        matched_id = -1;
    }
    
    dl_matrix3d_free(face_id);

    ESP_LOGI(TAG, "\nSimilarity: %.6f, id: %d", max_similarity, matched_id);

    return matched_id;
}

void add_face_id_extra(dl_matrix3d_t *dest_id,
                 dl_matrix3d_t *src_id)
{
    fptp_t *dest_item = dest_id->item;
    fptp_t *src_item = src_id->item;
    for (int i = 0; i < src_id->c; i++)
    {
       (*dest_item++) += (*src_item++);
    }
}
void devide_face_id_extra(dl_matrix3d_t *id, uint8_t num)
{
    fptp_t *in1 = id->item;
    for (int i = 0; i < id->c; i++)
    {
        (*in1++) /= num;
    }
}

int8_t enroll_face_extra(face_id_list *l, 
                dl_matrix3du_t *aligned_face)
{
    static int8_t confirm_counter = 0;
    dl_matrix3d_t *new_id = get_face_id(aligned_face);

    if ((l->count < l->size)&&(confirm_counter == 0))
        l->id_list[l->tail] = dl_matrix3d_alloc(1, 1, 1, FACE_ID_SIZE);

    add_face_id(l->id_list[l->tail], new_id);
    dl_matrix3d_free(new_id);

    confirm_counter++;

    if (confirm_counter == l->confirm_times)
    {
        devide_face_id_extra(l->id_list[l->tail], l->confirm_times);
        confirm_counter = 0;
        if(pickLocker == "LOCKER_1")
        {
          l->count = 1;
        }
        if(pickLocker == "LOCKER_2")
        {
          l->count = 2;
        }
        if(pickLocker == "LOCKER_3")
        {
          l->count = 3;
        }
        if(pickLocker == "LOCKER_4")
        {
          l->count = 4;
        }
        Serial.print("countFace: ");
        Serial.println(l->count);
        return 0;
    }

    return l->confirm_times - confirm_counter;
}
int8_t enroll_face_id_to_flash_extra(face_id_list *l,
              dl_matrix3du_t *aligned_face)
{
    int8_t left_sample = enroll_face_extra(l, aligned_face);
    if (left_sample == 0)
    {
        const esp_partition_t *pt = esp_partition_find_first(flash_type, flash_subtype, FR_FLASH_PARTITION_NAME);
        if (pt == NULL){
            ESP_LOGE(TAG, "Not found");
            return -2;
        }

        const int block_len = FACE_ID_SIZE * sizeof(float);
        const int block_num = (4096 + block_len - 1) / block_len;
        float *backup_buf = (float *)calloc(1, block_len);
        int flash_info_flag = FR_FLASH_INFO_FLAG;
        uint8_t enroll_id_idx = l->tail % l->size;

        if(enroll_id_idx % block_num == 0)
        {
             //save the other block TODO: if block != 2
            esp_partition_read(pt, 4096 + (enroll_id_idx + 1) * block_len, backup_buf, block_len);

            esp_partition_erase_range(pt, 4096 + enroll_id_idx * block_len, 4096);

            esp_partition_write(pt, 4096 + enroll_id_idx * block_len, l->id_list[enroll_id_idx]->item, block_len);
            //esp_partition_write(pt, 4096 + (enroll_id_idx + 1) * block_len, backup_buf, block_len); 
            Serial.print("SAVE ID + : ");
            Serial.println(4096 + enroll_id_idx * block_len);
        }
        else
        {
            // save the other block TODO: if block != 2
            esp_partition_read(pt, 4096 + (enroll_id_idx - 1) * block_len, backup_buf, block_len);

            esp_partition_erase_range(pt, 4096 + (enroll_id_idx - 1) * block_len, 4096);

            esp_partition_write(pt, 4096 + (enroll_id_idx - 1) * block_len, backup_buf, block_len);
            esp_partition_write(pt, 4096 + enroll_id_idx * block_len, l->id_list[enroll_id_idx]->item, block_len); 
            Serial.print("SAVE ID - : ");
            Serial.println(4096 + enroll_id_idx * block_len);
        }

        esp_partition_erase_range(pt, 0, 4096);
        esp_partition_write(pt, 0, &flash_info_flag, sizeof(int));
        esp_partition_write(pt, sizeof(int), l, sizeof(face_id_list));

        return 0;
    }

    return left_sample;
}
