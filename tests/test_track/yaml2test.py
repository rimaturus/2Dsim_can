import yaml

def yaml_to_txt(input_yaml_path, output_txt_path):
    """
    Converts a YAML file of cones to a text file in the specified format.
    
    Example of expected YAML structure:
      cones:
        - color: yellow
          x: 18.4878
          y: 0.0375
        - color: blue
          x: 19.1170
          y: 3.0
    """
    # Load the YAML data
    with open(input_yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    
    cones = data.get("cones", [])
    
    # Open the output file for writing
    with open(output_txt_path, 'w') as out_file:
        for i, cone in enumerate(cones):
            color_en = cone.get("color", "")
            
            # Map English color to Italian
            if color_en.lower() == "yellow":
                color_it = "giallo"
            elif color_en.lower() == "blue":
                color_it = "blu"
            else:
                # Fallback if color is missing or not recognized
                color_it = color_en
            
            x_val = cone.get("x", 0)
            y_val = cone.get("y", 0)
            
            out_file.write(f"points[{i}].x = {float(x_val)};\n")
            out_file.write(f"points[{i}].y = {float(y_val) + 10};\n")
            out_file.write(f"points[{i}].color = {color_it};\n\n")

if __name__ == "__main__":
    # Example usage: read from 'input.yaml' and write to 'output.txt'
    yaml_to_txt("/home/edo/unipi/1anno/istr/2Dsim_can/track/converted_circuit.yaml", "output.txt")
