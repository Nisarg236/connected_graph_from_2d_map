import yaml

# Load the original YAML data
with open("connected_graph_high_level.yaml", "r") as f:
    data = yaml.safe_load(f)

# Convert the data to the desired format
waypoints_cpp = []
for node_num, node_data in data.items():
    node_pos = node_data["node_pos"]
    edges = [{"outward_edge_to": str(edge_num), "path_length": str(distance)} for edge_num, distance in node_data["outward_edges"].items()]
    node_name = int(node_num)
    waypoints_cpp.append({"node_name": node_name,"node_pos": node_pos, "out_edges": edges})

# Save the converted YAML data
with open("converted_graph.yaml", "w") as f:
    yaml.dump({"waypoints_cpp": waypoints_cpp}, f, sort_keys=True)
