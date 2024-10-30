import re
import os
from typing import List, Tuple, Dict, Optional
from pathlib import Path
import argparse
import sys

class FormatConverter:
    def __init__(self):
        # Updated regex pattern to be more flexible with whitespace
        self.ry_pattern = re.compile(
            r"C\.addFrame\([\"'](?P<name>\w+)[\"']\)\s*\\\s*"  # Handle space after backslash
            r"\.setPosition\(\[(?P<position>[^]]+)\]\)\s*\\\s*"
            r"\.setShape\(ry\.ST\.(?P<shape>\w+),?\s*(?:size=)?\[(?P<size>[^]]+)\]\)\s*\\\s*"
            r"\.setColor\(\[(?P<color>[^]]+)\]\)\s*\\\s*"
            r"\.setContact\((?P<contact>\d+)\)"
        )
        

        # Input file boilerplate
        self.input_boilerplate_header = '''import robotic as ry
C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
'''        
        # Output file boilerplate
        with open('data/boilerplate_header.txt', 'r') as file:
            self.output_boilerplate_header = file.read()
        with open('data/boilerplate_footer.txt', 'r') as file:
            self.output_boilerplate_footer = file.read()

    def parse_vector(self, vector_str: str) -> List[float]:
        """Parse a vector string into a list of floats."""
        return [float(x.strip()) for x in vector_str.split(',')]

    def extract_object_definition(self, input_code: str) -> str:
        """Extract the object definition from the input code, ignoring boilerplate."""
        # Look for the object definition pattern anywhere in the input
        match = self.ry_pattern.search(input_code)
        if not match:
            # Print the input code for debugging
            print("Debug - Input code:")
            print(input_code)
            print("\nDebug - Regex pattern:")
            print(self.ry_pattern.pattern)
            raise ValueError("Could not find valid object definition in input file")
        return match.group(0)

    def ry_to_sim_utils(self, input_code: str) -> str:
        """Convert ry format to sim_utils format."""
        # Clean up input code by normalizing line endings
        input_code = input_code.replace('\r\n', '\n')
        
        # Extract just the object definition
        object_def = self.extract_object_definition(input_code)
        match = self.ry_pattern.match(object_def)
        
        if not match:
            raise ValueError("Invalid input format")
            
        data = match.groupdict()
        
        # Extract values
        name = data['name']
        position = self.parse_vector(data['position'])
        size = self.parse_vector(data['size'])
        color = self.parse_vector(data['color'])
        contact_enabled = bool(int(data['contact']))
        
        # Create the object definition with proper formatting
        object_template = '''
    # Test cube
    cfg_cuboid = sim_utils.MeshCuboidCfg(
        size={size},
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color={color}
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=False
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled={collision}
        )
    )

    cfg_cuboid.func(
        "/World/Origin1/Test",
        cfg_cuboid,
        translation={position}
    )'''
        
        formatted_object = object_template.format(
            size=tuple(size),
            color=tuple(color),
            collision=contact_enabled,
            position=tuple(position)
        )
        
        # Combine with boilerplate
        return self.output_boilerplate_header + "\n" + formatted_object + "\n" + self.output_boilerplate_footer
 
    def convert_file(self, input_path: str, output_path: str) -> None:
        """
        Convert a file from ry format to sim_utils format.
        
        Args:
            input_path: Path to the input file
            output_path: Path where the converted code should be saved
        """
        try:
            # Read input file
            with open(input_path, 'r') as f:
                input_code = f.read()
            
            # Convert the code
            converted_code = self.ry_to_sim_utils(input_code)
            
            # Write output file
            with open(output_path, 'w') as f:
                f.write(converted_code)
                
            print(f"Successfully converted {input_path} to {output_path}")
            
        except FileNotFoundError:
            print(f"Error: Could not find input file {input_path}")
            sys.exit(1)
        except PermissionError:
            print(f"Error: Permission denied when accessing {input_path} or {output_path}")
            sys.exit(1)
        except ValueError as e:
            print(f"Error: Invalid format in input file: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error: An unexpected error occurred: {e}")
            sys.exit(1)

    def create_template_input(self, output_path: str) -> None:
        """Create a template input file with boilerplate code."""
        template = self.input_boilerplate_header + '''C.addFrame('box') \\
 .setPosition([-.35,.1,1.]) \\
 .setShape(ry.ST.box, size=[.06,.06,.06]) \\
 .setColor([1,.5,0]) \\
 .setContact(1)
'''
        try:
            with open(output_path, 'w') as f:
                f.write(template)
            print(f"Created template input file: {output_path}")
        except Exception as e:
            print(f"Error creating template file: {e}")
            sys.exit(1)

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Convert robot object definition formats')
    parser.add_argument('input_file', help='Path to the input file containing the ry format code')
    parser.add_argument('output_file', help='Path where the converted sim_utils format code should be saved')
    parser.add_argument('--overwrite', action='store_true', help='Overwrite output file if it exists')
    parser.add_argument('--create-template', action='store_true', help='Create a template input file')
    parser.add_argument('--debug', action='store_true', help='Print debug information')
    
    args = parser.parse_args()
    
    converter = FormatConverter()
    
    # Handle template creation
    if args.create_template:
        if os.path.exists(args.input_file) and not args.overwrite:
            print(f"Error: File {args.input_file} already exists. Use --overwrite to overwrite it.")
            sys.exit(1)
        converter.create_template_input(args.input_file)
        sys.exit(0)
    
    # Check if output file exists and handle overwrite flag
    if os.path.exists(args.output_file) and not args.overwrite:
        print(f"Error: Output file {args.output_file} already exists. Use --overwrite to overwrite it.")
        sys.exit(1)
    
    # Convert file
    converter.convert_file(args.input_file, args.output_file)

if __name__ == "__main__":
    main()