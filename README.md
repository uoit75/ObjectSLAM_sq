### This is the code for paper: 
**Monocular Object SLAM Using Instance Segmentation with Superquadric Landmarks**.

The formulas from the paper can be found in the code comments labeled as **//Eq.xx**.


### Prerequisites
We have tested the code in **Ubuntu 18.04**, **Pangolin 0.5**, **OpenCV 3.4.15**, **Eigen 3.3.4**. This code is built based on [**ORB-SLAM3**](https://github.com/UZ-SLAMLab/ORB_SLAM3), so you can refer to its official environment requirements.


### Building
```
chmod +x build.sh       
./build.sh
```

### Examples

We provide a demo that uses the **TUM rgbd_dataset_freiburg3_long_office_household** sequence, please download the dataset in advance. We provide the offline instance segmentation results for this demo, which are located in `./Examples/Monocular/fr3_office_labels ` folder.

You can run the demo using the following command:
```
./Examples/Monocular/mono_tum ./Vocabulary/ORBvoc.txt ./Examples/Monocular/TUM3.yaml [path for fr3_office dataset]
```