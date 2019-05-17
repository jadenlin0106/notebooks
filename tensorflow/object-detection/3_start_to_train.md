## Training

**My environment:**

tensorflow-gpu: 1.12.2

cuda: 9.0.167

python: 3.5.2



After [2_convert_xml2csv2record](./2_convert_xml2csv2record.md), you should have your tfrecord files (training and testing).

Move to tensorflow/models/research :

```shell
$ mkdir ssd_model
$ mv [Your train.record] [Your test.record] ./ssd_model
```

Create your own `pascal_label_map.pbtxt` file in ./ssd_model like:

```pbtxt
item {
  id: 1
  name: 'class1'
}

item {
  id: 2
  name: 'class2'
}

item {
  id: 3
  name: 'class3'
}
	.
	.
	.

```

Download pre-trained model from [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md), and extract it like this:

```
$ tar -xzvf ssd_mobilenet_v1_coco.tar.gz --directory ./ssd_model
```

Also, download it's config file from [here](https://github.com/tensorflow/models/tree/master/research/object_detection/samples/configs), modify it:

```config
num_classes: 2 → number of class

batch_size: 8
→ recommend 8-12, reduce this number if your GPU ram is not enough.

fine_tune_checkpoint: "/home/digits/works/Mobilenet.projects/pre-trained-model/ssd_mobilenet_v2_coco_2018_03_29/model.ckpt"
→path to the pre-trained model, put "/model.ckpt" at the tail.

num_steps: 200000
→When will it stop, or you can just use ctrl+c

train_input_reader: {
  tf_record_input_reader {
    input_path: "/models/research/object_detection/ssd_model/train.record"
        →Path to train.record.

eval_input_reader: {
  tf_record_input_reader {
    input_path: "/models/research/object_detection/ssd_model/test.record"
        →Path to test.record.

label_map_path: "/models/research/object_detection/ssd_model/pascal_label_map.pbtxt"
→Path to pascal_label_map.pbtxt
```

Now, your ssd_model directory should have:

![1557901972787](./images/ssd_model_ls.png)

Move to tensorflow/models/research :

```shell
$ export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
```

If you didn't turn off your terminal after  [1_getting_started](./1_getting_started.md), then you don't need this command above.

Start to train:

```shell
$ python object_detection/legacy/train.py --train_dir object_detection/train --pipeline_config_path object_detection/ssd_model/ssd_mobilenet_v1_pets.config

#for GPU training
$ CUDA_VISIBLE_DEVICES=0,1 python object_detection/legacy/train.py --logtostderr --train_dir object_detection/train --pipeline_config_path object_detection/ssd_model/ssd_mobilenet_v1_pets.config
```

--train_dir: checkpoint, model.ckpt.....output's directory.

--pipeline_config_path: Path of config file.

It will start to train.

![1557904357837](./images/Itistraining.png)



Evaluate:

```
$ CUDA_VISIBLE_DEVICES=0,1 python object_detection/legacy/eval.py   --logtostderr   --pipeline_config_path=object_detection/ssd_model/ssd_mobilenet_v1_pets.config   --checkpoint_dir=object_detection/train --eval_dir=object_detection/eval
```



Monitor it by **tensorboard**:

```shell
$ tensorboard --logdir /models/research/object_detection/train/
```

and access [Host IP:port] on your browser.





Note:

If you encounter this while you are starting to train: 

```
............ImportError: libcublas.so.10.0: cannot open shared object file: No such file or directory
```

Check your tensorflow-gpu version:

```shell
$ pip show tensorflow
#or
$ pip show tensorflow-gpu
```

and check CUDA version:

```shell
$ cat /usr/local/cuda/version.txt
#or
$ nvcc --version
```

Compatible table: 

![1557912961580](./images/tensorflowCUDAtable.png)

from: https://www.tensorflow.org/install/source#gpu_support_2