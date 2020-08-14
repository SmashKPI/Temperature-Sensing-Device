'''
:Name:      raw_tcp_server.PY
:Author:    DTsebrii
:Date:      11/MAR/2020
:Description:Template for a raw TCP server    
'''
# *** Imports ********************************************************
import socket
# *** Constants ******************************************************
BUFFER_SIZE = 256  # Maximum 256 characters to receive
PORT = 1  # Insert port number here
HOST = ""  # Insert your socket ip here
LISTENERS = 2
final_msg_lst = []
part_msg = ''
temp_msg = ''
check_sum = 0
# *** Global Variables ***********************************************
# Server variables
serv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # AF_INET is for IPv4 and SOCK is for rosponce to TCP
serv_sock.bind((HOST, PORT))  
serv_sock.listen(LISTENERS)  # Two projects to listen from

def parse_sentence ():
    for index in range(len(message)):
        if message[ index ] == ',' or message[ index ] == '#':
            final_msg_lst.append( part_msg )
            part_msg = ''
        else:
            part_msg += message[ index ]

def validate_sentence ():
    for index in range(len(final_msg_lst)-1):
        for chank in final_msg_lst[index]:
            temp_msg += chank
        temp_msg += ','
    for elem in temp_msg:
        check_sum ^= ord(elem)
    if check_sum == int(final_msg_lst[-1]):
        print("Validation is ended. Storing process will begin immidiately")
        return True
    else
        raise BaseException
    
print("Server is ready to send data")  #Checking the server readiness
if __name__ == "__main__":
    try:
        while True:
            (clientsocket, address) = serv_sock.accept()  # Accept any client
            print("Connection has been established.")  # To check a connection
            message = clientsocket.recv(BUFFER_SIZE)  # Receiving of the message from a client
            message = str(message)
            message = message.replace("b'","")
            message = message.replace("'","")
            print(f"Message Received: {message}")
            parse_sentence()
            if validate_sentence():
                with open("demoFile.csv", "a") as csv_file:
                    csv_file.write(str(final_msg_lst[len(final_msg_lst) - 1])+'\n')
                    print("Message is stored")
            clientsocket.close()  # Close the connection
    except:
        print("Errorness connection\n")
        clientsocket.close()
    finally:
        serv_sock.close()   # to close the server
