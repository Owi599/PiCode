import socket

#This is a class for the communication interface from the RPi side (UDP)

class UDP:
	def __init__(self,ip,port):
		self.ipAddress = ip
		self.portNumber = port

	def create_server(self):
		SERVERSOCKET = socket.socket(family=socket.AF_INET, type = socket.SOCK_DGRAM)
		SERVERSOCKET.bind((self.ipAddress,self.portNumber))
		SERVERSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
		SERVERSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024*1024)	
		return SERVERSOCKET
			
	def create_client(self):
		CLIENTSOCKET = socket.socket(family=socket.AF_INET,type =socket.SOCK_DGRAM)
		CLIENTSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
		CLIENTSOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024*1024)
		return CLIENTSOCKET
		
	def send_data(self,data,client):
			client.sendto(str.encode(data),(self.ipAddress,self.portNumber))
	
	def receive_data(self,server):
		msgFromServer = server.recvfrom(1024)
		data = msgFromServer[0]
		receivedData = data.decode()
		return receivedData


#This is a class for the communication interface from the RPi side (TCP)


class TCP:
	def __init__(self,ip,port):
		self.ipAddress = ip
		self.portNumber = port

	def create_server(self):
		SERVERSOCKET = socket.socket(family=socket.AF_INET, type = socket.SOCK_STREAM)
		SERVERSOCKET.bind((self.ipAddress,self.portNumber))
		SERVERSOCKET.listen()
		return SERVERSOCKET
	
	def create_client(self):
		CLIENTSOCKET = socket.socket(family=socket.AF_INET,type =socket.SOCK_STREAM)
		CLIENTSOCKET.connect((self.ipAddress,self.portNumber))
		return CLIENTSOCKET
	
	def send_data(self,data,client):
		client.sendall(str.encode(data))
		
	def receive_data(self,server):
		connection , address = server.accept()
		data = connection.recv(1024)
		receivedData = data.decode()		
		return receivedData
