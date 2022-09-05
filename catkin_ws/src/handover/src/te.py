import yaml

# config = yaml.load('/config/multi_view.yaml', Loader=yaml.FullLoader)

with open('./config/multi_view.yaml') as f:
    config = yaml.load(f, Loader=yaml.FullLoader)

print(config['multi_view']['view_list'])
print(config['rotation_matrix']['rotation_matrix_left'])
