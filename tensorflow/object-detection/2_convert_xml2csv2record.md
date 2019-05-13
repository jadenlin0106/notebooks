## To use object detection model, you need tf_record file

Object detection's input is `.record` file, usually our raw dataset would be VOC format, so we have to convert.

#### My VOC & data architecture:

```
.
└── VOCdevkit                  #VOC's root
	├── models                 # tensorflow/models package
	├── protoc_3.3             # google protocol buffer tool
	└── traning_data           #datasets
		├── Annotations        # xml files, one on one map to JPEGImages's photos
		├── ImageSets          # txt files, contains photo path
		├── JPEGImages         # jpg files, photos
		├── csv_data           # csv files
		├── record_data        # tfrevoed files
		├── xml_to_csv.py
		└── generate_tfrecord.py
```

1. #### Step 1: convert xml to csv

   I assume that you have already finished [1_getting_started](./1_getting_started.md), cause we need the tensorflow/models package and export PYTHONPATH, also you will need tensorflow too:

   ```shell
   $ sudo pip install tensorflow
   ```

   To convert xml of VOC to csv file, use [xml_to_csv.py](./xml_to_csv.py).

   You may need to change `project_path` in `main()` of the code, and `value` in `xml_to_csv(path)` of the code.

   Usage:

   ```shell
   $ cd [Path to traning_data]
   $ python ./xml_to_csv.py
   ```

   Notice: My python version is 2.7.12

   Then, you will get a csv file called `train_labels.csv`. It contains 6 columns which are ['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax'].

   

2. #### Step 2: convert csv to record

   To convert csv to record, use [generate_tfrecord.py](./generate_tfrecord.py).

   You may need to change `flags.DEFINE_string` in the code, and customize you Classes in `class_text_to_int(row_label)` of the code.

   Usage example:

   ```shell
   $ python ./generate_tfrecord.py --csv_input=csv_data/train_labels.csv  --output_path=record_data/train.record
   ```

   It will take a while...

   

   Note:

   If you encounter something like:

   ```
   ..........tensorflow.python.framework.errors_impl.NotFoundError: ; No such file or directory
   ```

   means you didn't put the arguments `--csv_input=csv_data/train_labels.csv  --output_path=record_data/train.record`.

   

   If you encounter something like:

   ```
   .........return tf.train.Feature(int64_list=tf.train.Int64List(value=value))
   ```

   means you didn't assign your classes.

   

