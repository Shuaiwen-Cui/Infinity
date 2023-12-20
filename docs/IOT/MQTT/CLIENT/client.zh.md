# MQTT 客户端

## Paho客户端的Python实现

这是使用Paho-Python库创建MQTT客户端的示例。要运行，您需要首先安装依赖项。之后，您可以为订阅者和发布者创建两个脚本。先运行订阅者，然后运行发布者。您应该看到订阅者接收到的消息。

### 订阅者

```python
import paho.mqtt.client as mqtt

# Define the MQTT broker, topic, username, and password
broker_address = "8.222.194.160"  # Replace with your MQTT broker address
port = 1883 # Replace with your MQTT broker port
heartbeat_interval = 60  # The number of seconds until a heartbeat message is sent
topic = "csw-iot"
username = "cshwstem"  # Replace with your MQTT broker username
password = "88888888"  # Replace with your MQTT broker password

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to the topic when connected
    client.subscribe(topic)

# Callback when a message is received from the broker
def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

# Create an MQTT client instance
client = mqtt.Client()

# Set username and password
client.username_pw_set(username, password)

# Set callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(broker_address, port, heartbeat_interval)

# Start the loop to handle network communication, callbacks, and reconnecting
client.loop_start()

# Wait for the connection to be established
while not client.is_connected():
    pass

# Continue the program execution
try:
    # The client will subscribe to the topic in the on_connect callback
    while True:
        pass

except KeyboardInterrupt:
    # Disconnect from the broker when the program is interrupted
    client.disconnect()
    print("Disconnected from the broker")

```

### 发布者

```python
import paho.mqtt.client as mqtt

# Define the MQTT broker, topic, username, and password
broker_address = "8.222.194.160"  # Replace with your MQTT broker address
port = 1883 # Replace with your MQTT broker port
heartbeat_interval = 60  # The number of seconds until a heartbeat message is sent
topic = "csw-iot"
username = "cshwstem"  # Replace with your MQTT broker username
password = "88888888"  # Replace with your MQTT broker password

# Define the message to be sent
message = "Hello from Shuaiwen, MQTT!"

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Publish the message when connected
    client.publish(topic, message)

# Callback when the message is published
def on_publish(client, userdata, mid):
    print("Message Published")

# Create an MQTT client instance
client = mqtt.Client()

# Set username and password
client.username_pw_set(username, password)

# Set callback functions
client.on_connect = on_connect
client.on_publish = on_publish

# Connect to the broker
client.connect(broker_address, port, heartbeat_interval)

# Start the loop to handle network communication, callbacks, and reconnecting
client.loop_start()

# Wait for the connection to be established
while not client.is_connected():
    pass

# Continue the program execution
try:
    # The client will publish the message in the on_connect callback
    while True:
        pass

except KeyboardInterrupt:
    # Disconnect from the broker when the program is interrupted
    client.disconnect()
    print("Disconnected from the broker")

```

