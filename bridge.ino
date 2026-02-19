// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h> // Library for 3D coordinates (x, y, z)
#include <rclc/executor.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t pub_sonar;       // Publisher for Sonar distance
rcl_publisher_t pub_location;    // Publisher for Device Location (x, y, z)

// Subcriver
rcl_subscription_t sub_cmdV;

rclc_executor_t executor;

// Messages
std_msgs__msg__Float64 msg_sonar;
geometry_msgs__msg__Vector3 msg_pose;
geometry_msgs__msg__Twist cmd_vel;


// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x26, 0xEF, 0xD0};


// Must match the Sender's struct exactly
typedef struct {
  double distance_HCSR04;
  byte TAG_status[7];
  double posX_m;
  double posY_m;
  double posZ_m;
  byte quality;
} message_Robot;

message_Robot incomingReadings;

typedef struct {
  double linearX;
  double AngularZ;
} cmd_Vel;

cmd_Vel incomingSub;

// ========== FUNCTION PROTOTYPES ==========
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);


esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0;
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}

void OnDataRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
  // Copy received bytes into the local struct
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  // 1. Prepare and publish Sonar data
  msg_sonar.data = incomingReadings.distance_HCSR04;
  rcl_publish(&pub_sonar, &msg_sonar, NULL);

  // 2. Prepare and publish Location (Vector3)
  msg_pose.x = incomingReadings.posX_m;
  msg_pose.y = incomingReadings.posY_m;
  msg_pose.z = incomingReadings.posZ_m;
  rcl_publish(&pub_location, &msg_pose, NULL);

  // Optional Debug Serial Output
  Serial.printf("Recv: Sonar: %.2f | X: %.2f Y: %.2f\n", msg_sonar.data, msg_pose.x, msg_pose.y);
}

void VelocityCallBack(const void * msgin) {
  const geometry_msgs__msg__Twist * msg =
      (const geometry_msgs__msg__Twist *)msgin;

  incomingSub.linearX  = msg->linear.x;
  incomingSub.AngularZ = msg->angular.z;

  Serial.printf("Cmd Vel -> LinearX: %.2f | AngularZ: %.2f\n",
                incomingSub.linearX,
                incomingSub.AngularZ);

  esp_now_send(peerMAC, (uint8_t*)&incomingSub, sizeof(incomingSub));

}


void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}



// ========== SETUP FUNCTION ==========
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Initialize WiFi in Station mode
  WiFi.begin();
  // Match the WiFi channel with the sender
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register the receive callback function
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  addPeer(peerMAC, NULL);

  // Initialize micro-ROS transports
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create micro-ROS node
  rclc_node_init_default(&node, "Sonar_Location_node", "", &support);

  // 1. Initialize Sonar Distance Publisher
  rclc_publisher_init_default(
    &pub_sonar,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "distance_CRJG"
  );

  // 2. Initialize Device Location Publisher
  rclc_publisher_init_default(
    &pub_location,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "location_CRJG"
  );

  // Create subscriber
  rclc_subscription_init_default(
    &sub_cmdV,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmdV_CRJG"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmdV, &cmd_vel, &VelocityCallBack, ON_NEW_DATA);
}

// ========== MAIN LOOP ==========
void loop() 
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
 
  delay(1000);
}