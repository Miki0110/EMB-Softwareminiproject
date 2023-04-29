import os
import matplotlib.pyplot as plt

csv_folder = './csv_files'
output_folder = './plots'

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# List the CSV files in the folder
csv_files = [f for f in os.listdir(csv_folder) if f.endswith('.csv')]

# Loop through the CSV files
for csv_file in csv_files:
    input_values = []
    output_values = []

    # Read the data from the CSV file
    with open(os.path.join(csv_folder, csv_file), 'r') as f:
        for line in f:
            input_str, output_str = line.strip().split(',')
            input_value = float(input_str.split(':')[1])
            output_value = float(output_str.split(':')[1])
            input_values.append(input_value)
            output_values.append(output_value)

    # Plot the data
    plt.figure()
    plt.plot(input_values, label='Input')
    plt.plot(output_values, label='Output')
    plt.xlabel('Data Point')
    plt.ylabel('Value')
    plt.title(f'PID Results ({csv_file})')
    plt.legend()

    # Save the plot as a PDF file
    output_file = os.path.join(output_folder, f'{os.path.splitext(csv_file)[0]}.pdf')
    plt.savefig(output_file, format='pdf')
    plt.close()