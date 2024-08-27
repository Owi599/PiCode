import socket

#This is a class for the communication interface from the RPi side (UDP)

class UDP:
	def __init__(self,IP,PORT):
		self.IP_addr = IP
		self.PORT_num = PORT

	def CreateServer(self):
		ServerSocket = socket.socket(family=socket.AF_INET, type = socket.SOCK_DGRAM)
		ServerSocket.bind((self.IP_addr,self.PORT_num))
		ServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
		ServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024*1024)	
		return ServerSocket
			
	def CreateClient(self):
		ClientSocket = socket.socket(family=socket.AF_INET,type =socket.SOCK_DGRAM)
		ClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
		ClientSocket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024*1024)
		return ClientSocket
		
	def SendData(self,data):
			self.CreateClient().sendto(str.encode(data),(self.IP_addr,self.PORT_num))
	
	def RecData(self):
		msgfromServer = self.CreateServer().recvfrom(1024)
		data = msgfromServer[0]
		Recv_Data = data.decode()
		return Recv_Data


#This is a class for the communication interface from the RPi side (TCP)


class TCP:
	def __init__(self,IP,PORT):
		self.IP_addr = IP
		self.PORT_num = PORT

	def CreateServer(self):
		ServerSocket = socket.socket(family=socket.AF_INET, type = socket.SOCK_STREAM)
		ServerSocket.bind((self.IP_addr,self.PORT_num))
		ServerSocket.listen()
	
	def CreateClient(self):
		ClientSocket = socket.socket(family=socket.AF_INET,type =socket.SOCK_STREAM)
		ClientSocket.connect((self.IP_addr,self.PORT_num))
	
	def SendData(self,data):
		self.CreateClient().sendall(str.encode(data))
		
	def RecData(self):
		conn , addr = self.CreateServer().accept()
		data = conn.recv(1024)
		Recv_Data = data.decode()		
		return Recv_Data
