# Termex
---
Results:

***
For `blob_size=30`, `distance=(6,8)`:  

![a3068.th.gif](gifs/a3068.gif)

***
For `blob_size=40`, `distance=(6,8)`:  

![a3068.th.gif](gifs/a4068.gif)

***
For `blob_size=44`, `distance=(10,12)`:  

![a3068.th.gif](gifs/a441012.gif)

---
FirstNode is *termexLeptonCapture* to run this node use (get image from camera and communication with them):
```
rosrun [pkg_name] termexLeptonCapture 

parameters:
  freq - frequency of getting image (default 8)
```
For example:
```
rosrun termex termexLeptonCapture _freq:=6
```

If u want use this node to send converted image u should modify termexLeptonCapture.cpp unncoment SCALED_IMAGE & RESIZE if u need.

To communicate use topic `comand_console` type message std_msg/String. 
Example reset FFC: 
```
rostopic pub -1 comand_console std_msgs/String "FFC"
```

Raw image is published to topic `/lepton_output`, scaled image to `/lepton_scaled`.

---
Node witch calculate relocation of blob `termexRelocationBlobs` subscribe `/lepton_output`, and published `/relocation_array`.

Important field:
`blob_size` - blob size
`distance`  - distance between blobs

---


