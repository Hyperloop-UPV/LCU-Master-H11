import sys
from Packet_generation.Packet_generation import *

if len(sys.argv)<2:
    print("Please enter a board name,exiting...")
    sys.exit()
    
JSONpath = "Core/Inc/Code_generation/JSON_ADE"
aux = sys.argv[1]
filtered = ""
for char in aux:
    if char.isalpha():
        filtered += char
    else:
        break
board = filtered


boards = Generate_PacketDescription(JSONpath, board)

if __name__ == "__main__":
    Generate_DataPackets_hpp(board)
    Generate_OrderPackets_hpp(board)








        