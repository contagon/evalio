Loading robotics datasets can be tedious and time-consuming due to the lack of standardization in the datasets formats. evalio provides a unified interface for loading datasets, making it trivial to get up and running with real-world lidar and IMU data.

Datasets are managed via the CLI interface, specifically, using `evalio dl` to download datasets and `evalio rm` command to remove unwanted datasets. `evalio ls datasets` is also useful to visualize what has been installed. 

Once a dataset is downloaded, it can be used via the `evalio` library. Each dataset is an enum, with each sequence being a member.

```python
from evalio.datasets import Hilti2022

data = Hilti2022.basement_2
```

Each datasets provides helpers for iterating over data,

```python
from evalio.datasets import Hilti2022

# Iterate through all data in the dataset
for mm in Hilti2022.basement_2:
    print(mm)

# Access only lidar scans
for scan in Hilti2022.basement_2.lidar():
    print(scan)

# Access only IMU data
for imu in Hilti2022.basement_2.imu():
    print(imu)
```

Each time an iteration function is called, it will reload the data from the beginning. This obviously has a performance cost, so use wisely.

Helpers also exist for just gathering a single data point,

```python
from evalio.datasets import Hilti2022

# Get the 10th IMU data point
imu = Hilti2022.basement_2.get_one_imu(10)

# Get the 10th lidar data point
lidar = Hilti2022.basement_2.get_one_lidar(10)
```

!!! warning

    There is a cost to these single data point functions; at the moment this will cause a full iteration over the dataset to find the data point. This is a known issue and will be fixed in the future.

Each lidar measurement (type [LidarMeasurement][evalio.types.LidarMeasurement]) consists of a stamp and a vector of points (type [Point][evalio.types.Point]). The measurement follows a meticulous order, and always adhered to the following properties,

- The measurement stamp is always at the start of the scan
- Scans are in row-major order
- Scans contain all points, i.e. invalid points are NOT dropped. This is important information that is used in some algorithms for feature extractions. For example, a 128x1024 scan will contain 128x1024 points
- Points stamps are always relative to the start of the scan, so absolute point times can be calculated via `scan.stamp + scan.points[i].stamp`

Points are then easy to work with, as can be seen below.
```python
from evalio.datasets import Hilti2022
import matplotlib.pyplot as plt
import numpy as np

scan = Hilti2022.basement_2.get_one_lidar(10)

# Extract x, y, z coordinates
x = np.array([p.x for p in scan.points])
y = np.array([p.y for p in scan.points])
z = np.array([p.z for p in scan.points])

# Plot the bird's-eye view
plt.scatter(x, y, c=z, s=1)
plt.axis('equal')
plt.title("Bird's-Eye View of Point Cloud")
plt.show()
```

Additionally, a conversion to rerun types is also available for easy visualization, see [rerun][evalio.rerun] for more details.

```python
import rerun as rr
from evalio.rerun import convert

# Initialize rerun
rr.init("evalio")
rr.connect_tcp()

# Stream lidar scans to rerun
for scan in Hilti2022.basement_2.lidar():
    rr.set_time("timeline", timestamp=scan.stamp.to_sec())
    rr.log("lidar", convert(scan, color="z"))
```

If anything is unclear, please open an issue on the evalio repository. We are always looking to improve the documentation and make it easier to use.