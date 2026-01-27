import socket
import time


# This class implements a UDP-based communication interface on the RPi side.
# It provides helper methods to create sockets and measure send/receive times.
class UDP:
    def __init__(self, ip, port):
        """
        Initialize the UDP interface with target IP and port.

        Args:
            ip (str): IP address to bind/send to.
            port (int): UDP port number.
        """
        self.ipAddress = ip
        self.portNumber = port

    def create_server(self):
        """
        Create and configure a UDP server socket.

        Returns:
            socket.socket: Bound UDP socket ready to receive datagrams.
        """
        SERVERSOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # Bind to given IP and port to receive data
        SERVERSOCKET.bind((self.ipAddress, self.portNumber))
        # Increase receive and send buffer sizes (helps with high throughput / bursty traffic)
        SERVERSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        SERVERSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
        return SERVERSOCKET

    def create_client(self):
        """
        Create and configure a UDP client socket.

        Returns:
            socket.socket: UDP socket for sending datagrams.
        """
        CLIENTSOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # Match buffer settings with server side
        CLIENTSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        CLIENTSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
        return CLIENTSOCKET

    def send_data(self, data, client):
        """
        Send a string message via UDP and measure the send call duration.

        Args:
            data (str): Payload to send (will be UTF-8 encoded).
            client (socket.socket): UDP socket used for sending.

        Returns:
            float: Time taken (in seconds) by the send call.
        """
        startTime = time.perf_counter()
        client.sendto(str.encode(data), (self.ipAddress, self.portNumber))
        endTime = time.perf_counter()
        sendTime = endTime - startTime
        return sendTime

    def receive_data(self, server):
        """
        Receive a UDP datagram and measure the blocking receive duration.

        Args:
            server (socket.socket): Bound UDP socket used for receiving.

        Returns:
            tuple:
                receivedData (str): Decoded received message.
                receiveTime (float): Time spent blocked in recvfrom (seconds).
        """
        startTime = time.perf_counter()
        # recvfrom returns (data, address); only data is used here
        msgFromServer = server.recvfrom(1024)
        data = msgFromServer[0]
        receivedData = data.decode()
        endTime = time.perf_counter()
        receiveTime = endTime - startTime
        return receivedData, receiveTime


# This class implements a TCP-based communication interface on the RPi side.
# It creates blocking server/client sockets for reliable, connection-oriented data transfer.
class TCP:
    def __init__(self, ip, port):
        """
        Initialize the TCP interface with target IP and port.

        Args:
            ip (str): IP address to bind/connect to.
            port (int): TCP port number.
        """
        self.ipAddress = ip
        self.portNumber = port

    def create_server(self):
        """
        Create and configure a TCP server socket.

        Returns:
            socket.socket: Listening TCP socket ready to accept connections.
        """
        SERVERSOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        SERVERSOCKET.bind((self.ipAddress, self.portNumber))
        SERVERSOCKET.listen()  # Start listening for incoming client connections
        return SERVERSOCKET

    def create_client(self):
        """
        Create and connect a TCP client socket to the configured server.

        Returns:
            socket.socket: Connected TCP client socket.
        """
        CLIENTSOCKET = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        CLIENTSOCKET.connect((self.ipAddress, self.portNumber))
        return CLIENTSOCKET

    def send_data(self, data, client):
        """
        Send a string message over an established TCP connection.

        Args:
            data (str): Payload to send (will be UTF-8 encoded).
            client (socket.socket): Connected TCP client socket.
        """
        # sendall blocks until all data has been sent or an error occurs
        client.sendall(str.encode(data))

    def receive_data(self, server):
        """
        Accept a TCP connection and receive one message from the client.

        Args:
            server (socket.socket): Listening TCP server socket.

        Returns:
            str: Decoded payload received from the client.
        """
        # Block until a client connects
        connection, address = server.accept()
        # Receive up to 1024 bytes from the client
        data = connection.recv(1024)
        receivedData = data.decode()
        return receivedData
