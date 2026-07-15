import argparse
from Packet_generation.Packet_generation import (
    Generate_PacketDescription,
    Generate_DataPackets_hpp,
    Generate_OrderPackets_hpp,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate packet headers from JSON_ADE"
    )
    parser.add_argument(
        "board", help="Board key from Core/Inc/Code_generation/JSON_ADE/boards.json"
    )
    parser.add_argument(
        "adj", help="Path to JSON_ADE directory"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    json_path = args.adj.strip()
    board = args.board.strip()
    if not board:
        raise SystemExit("Board name cannot be empty")
    if not json_path:
        raise SystemExit("Adjustment configuration path cannot be empty")

    Generate_PacketDescription(json_path, board)
    Generate_DataPackets_hpp(board)
    Generate_OrderPackets_hpp(board)


if __name__ == "__main__":
    main()
