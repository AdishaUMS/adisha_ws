import yaml



def main(args=None):
    # Define path to test.yaml
    TEST_YAML_PATH = 'src/adisha_test/config/test.yaml'

    # Parse the file
    with open(TEST_YAML_PATH, 'r') as file:
        test_yaml = yaml.safe_load(file)

    # Print a value
    print(test_yaml['profil_robot']['tinggi'])



if __name__ == '__main__':
    main()