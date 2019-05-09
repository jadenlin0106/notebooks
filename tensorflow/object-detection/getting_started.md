## Getting started

Using TensorFlow with docker, object detection example.

#### step

1. Download docker:

   https://github.com/jadenlin0106/notebooks/blob/master/docker/install_docker.md



2. Create a container

   It will download docker image from Internet and run:

   ```shell
   $ docker run --runtime=nvidia --name jaden-tensorflow [-v host path:container path] -p 49152:8888 -p49153:22 -p 49154:6006 -it tensorflow/tensorflow:1.12.0-gpu-py3 bash
   ```

   - `--runtime=nvidia` : Container can use GPU resource.
   - `--name jaden-tensorflow` : Container name.
   - `-v host path:container path` : Mount host disk to container.
   - `-p 49152:8888` : Map port (Host:Container). 8888 is for "jupyter-notebook", 6006 is for "tensorboard", 22 is for "ssh"
   - `-it tensorflow/tensorflow:1.12.0-gpu-py3 bash` : Download image and interact with "bash". About images and tag, refer:
     - https://hub.docker.com/r/tensorflow/tensorflow
     - https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html
     - https://www.tensorflow.org/install/source

   

3. Install some packages:(Now, the terminal is in the container)

   ```shell
   $ apt-get update
   $ apt-get install git
   $ apt-get install wget
   ```

4. Down the model

   ```shell
   $ cd /
   $ git clone https://github.com/tensorflow/models.git
   ```

   

5. Install google protocol buffer tool `protoc`, and compile the code:

   ```shell
   $ mkdir protoc_3.3
   
   # Download and unzip protoc 3.3
   $ cd protoc_3.3
   $ wget https://github.com/google/protobuf/releases/download/v3.3.0/protoc-3.3.0-linux-x86_64.zip
   $ chmod 775 protoc-3.3.0-linux-x86_64.zip
   $ unzip protoc-3.3.0-linux-x86_64.zip
   $ cd /models/research/
   
   # Compile /models/research/object_detection/protos/*.proto with protoc
   $ /protoc_3.3/bin/protoc object_detection/protos/*.proto --python_out=.
   ```

   The followings are optional, I tried work without them:

   ```shell
   $ pip install pillow
   $ pip install lxml
   $ pip install jupyter
   $ pip install matplotlib
   
   # add models/research and models/research/slim in PYTHONPATH
   $ export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
   ```

6. Open jupyter notebook:

   ```shell
   $ cd /
   $ ./run_jupyter.sh
   ```

   Open your browser access [You host IP:49152], put the token shows on terminal.

   Get in /models/research/object_detection/, click `object_detection_tutorial.ipynb` to run.

   

7. Test the code:

   Change the code like this:

   ```
   PATH_TO_TEST_IMAGES_DIR = 'test_images'
   TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, 'image{}.jpg'.format(i)) for i in range(1, 8) ]
   ```

    The path is located at:

   /models/research/object_detection/test_images

