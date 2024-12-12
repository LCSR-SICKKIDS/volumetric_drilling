def process_list(input_list):
    processed_list = []
    
    for item in input_list:
        # Check if the item contains a comma
        if ',' in item:
            # Split the item by the first comma and add a comma to the start of the second element
            part1, part2 = item.split(',', 1)
            processed_list.append(part1)
            processed_list.append(',' + part2)  # Keep the comma in the second part
        else:
            # If no comma, just add the item as is
            processed_list.append(item)
    
    return processed_list

# Example usage
input_list = [
    '--launch_file', '/home/amunawa2/volumetric_drilling_VRE/launch.yaml', '-l', '0,7', '-a', 
    '/home/amunawa2/volumetric_drilling_VRE/ADF/volume_256_RT143.yaml', '--plugins', 
    '/home/nimesh/ambf_spacenav_plugin/build/libspacenav_plugin.so,/home/amunawa2/ambf_video_recording/build/libsimulator_video_recording.so', 
    '--spf', '/home/nimesh/ambf_spacenav_plugin/example/spacenav_config.yaml', '--fp', '/dev/input/js0'
]

# Call the function and get the processed list
processed_list = process_list(input_list)

# Print the result
print(processed_list)
