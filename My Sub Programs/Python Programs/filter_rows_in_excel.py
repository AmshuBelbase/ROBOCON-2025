import pandas as pd

# Load the Excel file
file_path = "odom_data.xlsx"  # Change this path if needed
xls = pd.ExcelFile(file_path)

# Load the sheet into a DataFrame
df = xls.parse('val1')  # Change 'val1' if your sheet has a different name

# Define the three columns to check for duplicates
# Replace with actual column names if different
columns_to_check = ["Distance LR", "Distance Front", "Angle"]

# Identify rows where all 3 columns match with the previous row
mask = (df[columns_to_check] == df[columns_to_check].shift(1)).all(axis=1)

# Keep only rows that do not match
df_filtered = df[~mask].reset_index(drop=True)

# Save the filtered DataFrame back to an Excel file
output_path = "filtered_odom_data.xlsx"
df_filtered.to_excel(output_path, index=False)

print(f"Filtered file saved at: {output_path}")
