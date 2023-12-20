# Message Queuing Telemetry Transport (MQTT)

## What is MQTT?

MQTT is a lightweight publish/subscribe messaging protocol designed for M2M (machine to machine) telemetry in low bandwidth environments. It was invented by Dr. Andy Stanford-Clark (IBM) and Arlen Nipper in 1999 and became an OASIS standard in 2014.

## MQTT Features

- **Lightweight**: MQTT is designed to be used in low-bandwidth, high-latency networks. It is a publish/subscribe, extremely simple and lightweight messaging protocol, designed for constrained devices and low-bandwidth, high-latency or unreliable networks. The design principles are to minimise network bandwidth and device resource requirements whilst also attempting to ensure reliability and some degree of assurance of delivery. These principles also turn out to make the protocol ideal of the emerging "machine-to-machine" (M2M) or "Internet of Things" world of connected devices, and for mobile applications where bandwidth and battery power are at a premium.

- **Publish/Subscribe**: MQTT is based on a publish/subscribe model. This makes it suitable for Internet of Things messaging such as with low power sensors or mobile devices such as phones, embedded computers or microcontrollers.

- **Asynchronous**: MQTT is an asynchronous protocol. An MQTT client can send a message and carry on with other processing, a message will be delivered to the client when it is available.

- **QoS**: MQTT supports three qualities of service for delivering messages between clients and servers: "at most once", "at least once" and "exactly once". It is possible to achieve reliability with the "at least once" or "exactly once" modes, but this is not the default. For example, a sensor might only need to send a reading when a value has changed but an actuator may need to receive every message to ensure it has the latest control information.

- **Retained Messages**: MQTT supports retained messages. This is a useful feature of the MQTT protocol. It allows a client to publish a message that is stored by the server and sent to any future subscribers that subscribe to the matching topic. This is useful for messages such as status updates where a client can publish an update to a status topic and any new clients that connect will receive the most recent update.

- **Small code footprint**: The whole MQTT protocol specification is documented in about 30 pages of text. This makes it ideal for mobile devices such as cell phones, which need to keep their power usage down, or for embedded devices where memory is at a premium.

- **Open protocol**: MQTT is an open protocol, and as such is royalty free. It is also an OASIS standard.

## Working Paradigm

MQTT is a publish/subscribe messaging protocol. It is designed for connections with remote locations where a "small code footprint" is required or the network bandwidth is limited. The publish/subscribe messaging pattern requires a message broker. The broker is responsible for distributing messages to interested clients based on the topic of a message.

### Key Roles

- **Publisher**: A publisher is an application that connects to a broker. It creates and sends messages. The content of the messages is completely up to the publisher. The broker will distribute the messages to interested subscribers.

- **Subscriber**: A subscriber is an application that connects to the broker. The subscriber can specify the topics it is interested in. The broker will distribute only the messages that are of interest to the subscriber.

- **Broker**: A broker is an intermediary that handles the passing of messages between publishers and subscribers. The Eclipse Mosquitto broker is open source and can be downloaded from the Mosquitto project page on GitHub.

## MQTT Servers

### EMQ X

EMQ X is an open source MQTT broker that implements MQTT versions 5.0, 3.1.1 and 3.1. It is a highly scalable broker that can be run in the cloud. It is an open source project and can be found on the EMQ X project page.

### Mosquitto

Mosquitto is an open source message broker that implements MQTT versions 5.0, 3.1.1 and 3.1. It is a lightweight broker that can be run on low power devices such as the Raspberry Pi. It is an open source project and can be found on the Eclipse Mosquitto project page.

### HiveMQ

HiveMQ is a commercial MQTT broker that implements MQTT versions 5.0, 3.1.1 and 3.1. It is a highly scalable broker that can be run in the cloud. It is a commercial product and can be found on the HiveMQ website.

## MQTT Clients - (Publishers and Subscribers)

### Paho

Paho is an open source MQTT client library. It is available for a wide range of programming languages including Java, JavaScript, C, C++, Python, Go, Ruby and C#. It is an open source project and can be found on the Paho project page.

### EMQ X

EMQ X is an open source MQTT client library. It is available for a wide range of programming languages including Java, JavaScript, C, C++, Python, Go, Ruby and C#. It is an open source project and can be found on the EMQ X project page.

### MQTT.js

MQTT.js is an open source MQTT client library. It is available for JavaScript. It is an open source project and can be found on the MQTT.js project page.





