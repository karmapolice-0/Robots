import re

input_path, output_path = "models/", "output_data/"
fn = input('Enter file name: ')

lines = []
try:
    file = open(input_path+fn, 'r')
    for line in file:
        lines.append(re.sub(',+', ' ', line))
    file.close()
except FileNotFoundError:
    print(f"File {fn} not found")

file1 = open(output_path+fn, 'w')
file1.writelines(lines)
file1.close()
print(f'Point Cloud txt updated and located in "../output_data/{fn}"')
