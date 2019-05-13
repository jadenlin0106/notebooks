import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET


def xml_to_csv(path):
    xml_list = []
    #print(str(path+'/*.xml'))
    files = os.listdir(path)
    for file in files:
        for xml_file in glob.glob(path + '/' + file + '/*.xml'):
                tree = ET.parse(xml_file)
                root = tree.getroot()
                for member in root.findall('object'):
                        value = ('JPEGImages/'+member[0].text,
                        #      int(root.find('size')[0].text),
                        #      int(root.find('size')[1].text),
                                '1920',
                                '1080',
                                member[1].text,
                                int(member[2][0].text),
                                int(member[2][1].text),
                                int(member[2][2].text),
                                int(member[2][3].text)
                                )
                        xml_list.append(value)
    column_name = ['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df


def main():
    for directory in ['All']:
        project_path = '/home/ros/sharefloder/training_data/Annotations'
        image_path = os.path.join(project_path, directory)
        xml_df = xml_to_csv(image_path)
        xml_df.to_csv('csv_data/train_labels.csv', index=None)
        print('Successfully converted xml to csv.')


main()