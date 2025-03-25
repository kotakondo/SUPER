#!/usr/bin/env python3
import os
import sys
import glob
import csv

def compute_averages(filepath):
    """
    Reads a CSV file (with header) and computes the average for each column.
    Returns a tuple (averages, counts) where:
      - averages is a dict mapping column name -> average value.
      - counts is a dict mapping column name -> number of valid samples.
    """
    sums = {}
    counts = {}
    with open(filepath, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key, value in row.items():
                try:
                    val = float(value.strip())
                except Exception:
                    continue
                sums[key] = sums.get(key, 0.0) + val
                counts[key] = counts.get(key, 0) + 1
    averages = {}
    for key in sums:
        if counts[key] > 0:
            averages[key] = sums[key] / counts[key]
        else:
            averages[key] = None
    return averages, counts

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <folder_path>".format(sys.argv[0]))
        sys.exit(1)
    
    folder = sys.argv[1]
    pattern = os.path.join(folder, "time_consuming_num_*.csv")
    files = glob.glob(pattern)
    if not files:
        print("No files found matching pattern in folder:", folder)
        sys.exit(1)
    
    overall_sums = {}
    overall_counts = {}
    results = []  # list of tuples: (filename, averages)
    
    for filepath in sorted(files):
        averages, counts = compute_averages(filepath)
        filename = os.path.basename(filepath)
        results.append((filename, averages))
        
        # Update overall sums and counts for each column
        for key in averages:
            overall_sums[key] = overall_sums.get(key, 0.0) + averages[key] * counts[key]
            overall_counts[key] = overall_counts.get(key, 0) + counts[key]
        
        print("Processed file: {}".format(filename))
    
    # Compute overall averages for each column
    overall_averages = {}
    for key in overall_sums:
        overall_averages[key] = overall_sums[key] / overall_counts[key]
    
    # Write human-readable output to a text file.
    output_txt = os.path.join(folder, "average_computation_times.txt")
    with open(output_txt, "w") as f:
        f.write("Average Computation Times Report\n")
        f.write("====================================\n\n")
        for filename, averages in results:
            f.write("File: {}\n".format(filename))
            for key in sorted(averages.keys()):
                if averages[key] is not None:
                    f.write("   {}: {:.6f}\n".format(key, averages[key]))
                else:
                    f.write("   {}: N/A\n".format(key))
            f.write("\n")
        
        f.write("Overall Averages:\n")
        f.write("====================================\n")
        for key in sorted(overall_averages.keys()):
            f.write("   {}: {:.6f}\n".format(key, overall_averages[key]))
    
    print("Results written to {}".format(output_txt))

if __name__ == "__main__":
    main()
