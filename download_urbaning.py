"""Download one UrbanING-V2X sequence or the complete dataset."""

import argparse

from urbaning.data import download_dataset, download_one_sequence


def main():
    """Parse download options and delegate archive handling to the UrbanING SDK."""
    parser = argparse.ArgumentParser(description="Download the UrbanING-V2X dataset.")
    parser.add_argument(
        "--all",
        action="store_true",
        help="Download the entire dataset instead of one sequence.",
    )
    parser.add_argument(
        "--sequence-name",
        help="Optionally choose which sequence to download.",
    )
    args = parser.parse_args()
    download_dir = "datasets/UrbanIng-V2X"

    if args.all:
        if args.sequence_name:
            parser.error("--sequence-name cannot be used with --all")
        download_dataset(download_dir=download_dir)
    elif args.sequence_name:
        download_one_sequence(
            download_dir=download_dir,
            sequence_name=args.sequence_name,
        )
    else:
        download_one_sequence(download_dir=download_dir)


if __name__ == "__main__":
    main()