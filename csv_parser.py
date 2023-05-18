import csv
import os

def parse_csv(filename, column1_header, time_header):
    # Variables to store column values
    column1_values = []
    column2_values = []

    # Read the CSV file
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)

        # Iterate over each row in the CSV file
        for row in reader:
            # Get the values from columns 1 and 2 using the header names
            column1_value = float(row[column1_header])
            column2_value = float(row[time_header])

            # Append the values to their respective lists
            column1_values.append(column1_value)
            column2_values.append(column2_value)

    # Check if the last entry in column1 is smaller than the previous entry
    if column1_values[-1] < column1_values[-2]:
        column1_values.pop()
        column2_values.pop()

    # Calculate the differences
    column1_diff = column1_values[-1] - column1_values[0]
    column2_diff = column2_values[-1] - column2_values[0]

    # Calculate the division result
    division_result = column1_diff / column2_diff

    return division_result


if __name__ == "__main__":

    directory = "./results/"

    foldername = "20230516_095616_diam2"
    column1_header = "x_q10_2"
    time_header = "time"

    # Loop over all the CSV files in the directory
    for filename in sorted(os.listdir(f"{directory}{foldername}/")):
        if filename.endswith(".csv"):
            division_result = parse_csv(f"{directory}{foldername}/{filename}", column1_header, time_header)
            print(f"The division result for {filename} is: {division_result}")
