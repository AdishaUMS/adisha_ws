import yaml



def main(args=None):
    # Define path to test.yaml
    TEST_YAML_PATH = 'src/adisha_test/config/test.yaml'

    # Parse the file
    with open(TEST_YAML_PATH, 'r') as file:
        test_yaml = yaml.safe_load(file)

    # Print a value
    print(test_yaml['profil_robot']['tinggi'])

    # Edit the file
    test_yaml['versi'] = '0.0.0'

    # Overwrite
    with open(TEST_YAML_PATH, 'w') as file:
        yaml.safe_dump(test_yaml, file)



if __name__ == '__main__':
    main()