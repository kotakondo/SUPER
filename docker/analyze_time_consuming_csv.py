#!/usr/bin/env python3
import os
import sys
import glob
import pandas as pd

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <folder_with_csv_files>".format(sys.argv[0]))
        sys.exit(1)

    folder = sys.argv[1]
    # Find CSV files with names like time_consuming_*.csv
    csv_files = glob.glob(os.path.join(folder, "time_consuming_*.csv"))
    if not csv_files:
        print("No CSV files found in folder:", folder)
        sys.exit(1)

    summary_lines = []
    summary_lines.append("Time Consuming Statistics:\n\n")
    
    all_data = []  # list to collect DataFrames from all files
    file_means = {}  # dictionary to store means per file

    # Sort files alphabetically (or you could sort numerically if needed)
    for csv_file in sorted(csv_files):
        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            print(f"Error reading file {csv_file}: {e}")
            continue

        # If the CSV has a column 'TIME_STAMPE', drop it for average computation.
        if 'TIME_STAMPE' in df.columns:
            df_numeric = df.drop(columns=['TIME_STAMPE'])
        else:
            df_numeric = df

        # Compute mean for each numeric column.
        means = df_numeric.mean()
        file_means[os.path.basename(csv_file)] = means
        all_data.append(df_numeric)

        summary_lines.append(f"File: {os.path.basename(csv_file)}\n")
        for col, val in means.items():
            summary_lines.append(f"  {col}: {val:.6f}\n")
        summary_lines.append("\n")
    
    # Combine all data for overall averages.
    combined = pd.concat(all_data, ignore_index=True)
    overall_means = combined.mean()
    summary_lines.append("Overall Averages Across All Files:\n")
    for col, val in overall_means.items():
        summary_lines.append(f"  {col}: {val:.6f}\n")
    
    # Save the statistics into a text file in the same folder.
    output_file = os.path.join(folder, "time_consuming_statistics.txt")
    with open(output_file, "w") as f:
        f.writelines(summary_lines)
    
    print("Saved statistics to:", output_file)

if __name__ == "__main__":
    main()
