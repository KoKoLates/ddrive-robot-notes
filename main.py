import argparse

def main() -> None:
    """"""

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--ctrl', type=str, default="pid")
    args = parser.parse_args()

    if args.ctrl not in ("pid", "mpc"):
        raise ValueError("The select type in not in ctrl list")

    print(f"[INFO] using controller: {args.ctrl}")
    main()
