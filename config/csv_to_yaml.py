import yaml
import pandas as pd
import numpy as np


# Read csv file
csv_fp = "demo.csv"
ur10e_path = pd.read_csv(csv_fp, header=None, index_col=None).values

concatenated_path = np.vstack((ur10e_path, ur10e_path[:-1][::-1]))


yaml_data = {
    "trajectory": {}
}

for i, data in enumerate(concatenated_path):
    yaml_data["trajectory"][i+1] = data.tolist()


# Save the path to yaml file
with open(f"{csv_fp[:-4]}.yaml", "w") as output_yaml:
    yaml.dump(yaml_data, output_yaml, default_flow_style=False)
