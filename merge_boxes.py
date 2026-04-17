#!/usr/bin/env python3
"""
Merges all box models in a Gazebo SDF file into a single model
with multiple links to reduce physics overhead.
"""

import xml.etree.ElementTree as ET
import sys
import os

def merge_boxes(input_file):
    output_file = input_file.replace('.sdf', '_merged.sdf')
    
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    world = root.find('world')
    if world is None:
        print("No world found in SDF")
        return

    # Collect all box models
    box_models = []
    non_box_models = []
    
    for model in world.findall('model'):
        name = model.get('name', '')
        if name.startswith('box_') or name.startswith('box'):
            box_models.append(model)
        else:
            non_box_models.append(model)
    
    print(f"Found {len(box_models)} box models to merge")
    
    if len(box_models) == 0:
        print("No box models found!")
        return

    # Create merged model
    merged_model = ET.Element('model')
    merged_model.set('name', 'ziekenhuis_walls')
    
    pose_elem = ET.SubElement(merged_model, 'pose')
    pose_elem.text = '0 0 0 0 0 0'
    
    static_elem = ET.SubElement(merged_model, 'static')
    static_elem.text = 'true'

    # Add each box as a link in the merged model
    for i, model in enumerate(box_models):
        model_pose = model.find('pose')
        model_pose_text = model_pose.text if model_pose is not None else '0 0 0 0 0 0'
        
        link = ET.SubElement(merged_model, 'link')
        link.set('name', f'wall_{i}')
        
        link_pose = ET.SubElement(link, 'pose')
        link_pose.text = model_pose_text
        
        # Find geometry from original model
        for orig_link in model.findall('link'):
            # Copy visual
            for visual in orig_link.findall('visual'):
                new_visual = ET.SubElement(link, 'visual')
                new_visual.set('name', f'visual_{i}')
                for child in visual:
                    new_visual.append(child)
            
            # Copy collision
            for collision in orig_link.findall('collision'):
                new_collision = ET.SubElement(link, 'collision')
                new_collision.set('name', f'collision_{i}')
                for child in collision:
                    new_collision.append(child)

    # Remove old box models and add merged model
    for model in box_models:
        world.remove(model)
    
    world.append(merged_model)
    
    # Write output
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='unicode', xml_declaration=True)
    
    print(f"Done! Saved to: {output_file}")
    print(f"Reduced {len(box_models)} models to 1 merged model")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 merge_boxes.py <path_to_sdf>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    if not os.path.exists(input_file):
        print(f"File not found: {input_file}")
        sys.exit(1)
    
    merge_boxes(input_file)
