# Internet of Things

![IOT](iot.jpeg){: style="height:400px;width:800px" }

## Overview

<div class="grid cards" markdown>
-  :material-book:{ .lg .middle } __IoT and Edge Computing for Architects - Second Edition🎯✅🏆__

    ---

    Arthor: Perry Lea

    [:octicons-arrow-right-24: <a href="https://learning.oreilly.com/library/view/iot-and-edge/9781839214806/" target="_blank"> Portal </a>](#)

</div>

## Hardware

Refer to the __Embedded System__ Tab. 

## Software

To be updated.

## Protocol

### Open Systems Interconnection (OSI) Model

![OSI](osi.png)

[original@sheldon](https://community.fs.com/article/tcpip-vs-osi-whats-the-difference-between-the-two-models.html)

!!! info
    OSI model is a conceptual model that characterizes and standardizes how different software and hardware components involved in a network communication should divide labor and interact with one another. It has seven layers.

#### Layer 7: Application Layer

The application layer of OSI model interacts directly with software applications to provide communication functions as required, and it is the closest to end users. Functions of application layer typically include verifying the availability of communication partners and resources to support any data transfer. This layer also defines protocols for end applications, such as domain name system (DNS), file transfer protocol (FTP), hypertext transfer protocol (HTTP), Internet massage access protocol (IMAP), post office protocol (POP), simple mail transfer protocol (SMTP), Simple Network Management Protocol (SNMP), and Telnet (a terminal emulation).

#### Layer 6: Presentation Layer

The presentation layer checks the data to ensure it is compatible with the communications resources. It translates the data into the form that the application level and lower levels accept. Any needed data formatting or code conversion is also handled by the sixth layer, such as converting an Extended Binary Coded Decimal Interchange Code (EBCDIC) coded text file to an American Standard Code for Information Interchange (ASCII) coded text file. It functions for data compression and encryption as well. For example, video calls will be compressed during the transmission so that it can be transmitted faster, and the data will be recovered at the receiving side. For the data that has high security requirements, such as a text message containing your password, it will be encrypted at this layer.

#### Layer 5: Session Layer

The session layer controls the dialogues (connections) between computers. It establishes, manages, maintains and ultimately terminates the connections between the local and remote application. Layer 5 software also handles authentication and authorization functions. It verifies the data is delivered as well. The session layer is commonly implemented explicitly in application environments that use remote procedure calls.

#### Layer 4: Transport Layer

The transport layer provides the functions and means of transferring data sequences from a source to a destination host via one or more networks, while maintaining the quality of service (QoS) functions and ensure the complete delivery of the data. The integrity of the data can be guaranteed via error correction and similar functions. It can also provide explicit flow control function. Though not strictly conforming to the OSI model, the TCP and User Datagram Protocols (UDP) are essential protocols in layer 4.

#### Layer 3: Network Layer

The network layer handles packet routing via logical addressing and switching functions. A network is a medium to which many nodes can be connected. Every node has an address. When a node needs to transfer message to other nodes, it can merely provide the content of the massage and the address of the destination node, then the network will find the way to deliver the message to the destination node, possibly routing through other nodes. If the message is too long, the network may split it into several segments at one node, sending them separately and reassembling the fragments at another node.

#### Layer 2: Data Link Layer

The data link layer provides node-to-node transfer—a link between two directly connected nodes. It handles packaging and unpacking the data in frames. It defines the protocol to establish and terminate a connection between two physically connected devices, such as Point-to-Point Protocol (PPP). The data link layer is generally divided into two sublayers—media access control (MAC) layer and logical link control (LLC) layer. MAC layer is responsible for controlling how devices in a network gain access to a media and permission to transmit data. LLC layer is responsible for identifying and encapsulating network layer protocols, and controls error checking and frame synchronization.

#### Layer 1: Physical Layer

The physical layer defines the electrical and physical specifications of the data connection. For example, the layout of pins of the connector, the operation voltages of an electrical cable, optical fiber cable specifications, and the frequency for wireless devices. It is responsible for transmission and reception of unstructured raw data in a physical medium. Bit rate control is done at the physical layer. It is the layer of low-level networking equipment and is never concerned with protocols or other higher-layer items.

### TCP/IP Model

!!! info
    TCP/IP model is also a layered reference model, but it is a four-layer model. Another name for it is Internet protocol suite. It is commonly known as TCP/IP because the foundational protocols are TCP and IP, but not only these two protocols are used in this model.

#### Application Layer

The application layer of TCP/IP model provides applications the ability to access to services of the other layers, and defines the protocols that applications use to exchange data. Most widely-known application layer protocols include HTTP, FTP, SMTP, Telnet, DNS, SNMP and Routing Information Protocol (RIP).

#### Transport Layer

The transport layer, also known as the host-to-host transport layer, is responsible for providing the application layer with session and datagram communication services. The core protocols of this layer are TCP and UDP. TCP provides a one-to-one, connection-oriented, reliable communications service. It is responsible for sequencing and acknowledgment of packets sent, and recovery of packets lost in transmission. UDP provides one-to-one or one-to-many, connectionless, unreliable communications service. UDP is used typically when the amount of data to be transferred is small (such as that data would fit into a single packet).

#### Internet Layer

The Internet layer is responsible for host addressing, packaging, and routing functions. The core protocols of the Internet protocol layer are IP, Address Resolution Protocol (ARP), Internet Control Message Protocol (ICMP) and Internet Group Management Protocol (IGMP). The IP is a routable protocol responsible for IP addressing, routing, and the fragmentation and reassembly of packets. The ARP is responsible for the discovering the network access layer address such as a hardware address associated with a given Internet layer access. The ICMP is responsible for providing diagnostic functions and reporting errors due to the unsuccessful delivery of IP packets. The IGMP is responsible for the management of IP multicast groups. In this layer, the IP adds header to the packets, which is known as IP address. Now there’s both IPv4 (32-bit) address and IP Ipv6 (128-bit) address.

#### Network Access Layer

Network access layer (or link layer) is responsible for placing the TCP/IP packets on the network medium and receiving TCP/IP packets off the network medium. TCP/IP is designed to be independent of the network access method, frame format, and medium. In other word, it is independent from any specific network technology. In this way, TCP/IP can be used to connect different network types, such as Ethernet, Token Ring, X.25, Frame Relay, and Asynchronous Transfer Mode (ATM).

![pipeline](pipeline.jpg)

![comparison](comparison.jpg)

### Protocol Stack

![protocol](protocol.gif)
[🔗 image source](https://ieeexplore.ieee.org/mediastore_new/IEEE/content/media/6287639/8948470/9090208/almas1-2993363-large.gif)

I will use the OSI model layer to classify the protocols. 

#### Layer 5: Session Layer & Layer 6: Presentation Layer & Layer 7: Application Layer

##### REST/HTTP

##### MQTT

##### CoAP

##### AMQP

##### DDS

##### XMPP

##### Modbus

#### Layer 3: Network Layer & Layer 4: Transport Layer

##### TCP

##### UDP

##### IPv4

##### IPv6

##### 6LoWPAN

#### Layer 1: Physical Layer & Layer 2: Data Link Layer

##### Bluetooth 📶

##### Thread 📶

##### WiFi(IEEE 802.11) 📶

##### Zigbee 📶

##### Z-Wave 📶

##### LoRa 📶

##### NB-IoT 📶

##### LTE-M 📶

##### Sigfox 📶

##### NFC 📶

##### RFID 📶

##### Ethernet🔌

##### USB🔌

### IoT Protocol Comparison

![comparison](comparison.png)




















## Framework

Home Assistant

ThingsBoard