from .base import EVALIO_DATA, Dataset, RosbagIter, PreintNoise, load_pose_csv
from rosbags.typesys import Stores
import gtsam


class NewerCollege2020(Dataset):
    def __init__(self, seq: str):
        if seq not in NewerCollege2020.sequences():
            raise ValueError(f"Sequence {seq} not found in {self.name()}")

        self.seq = seq

    def __iter__(self):
        RosbagIter(
            "/mnt/grizzly/newer2020/01_short_experiment/rosbag/rooster_2020-03-10-10-36-30_0.bag",
            Stores.ROS1_NOETIC,
            "/os1_cloud_node/points",
            "/os1_cloud_node/imu",
        )

    def ground_truth(self) -> list[gtsam.Pose3]:
        return load_pose_csv(
            EVALIO_DATA / NewerCollege2020.name() / self.seq / "ground_truth.csv"
        )

    def download(self):
        # TODO: Check if already downloaded
        folder_id = {
            "01_short_experiment": "1WWtyU6bv4-JKwe-XuSeKEEEBhbgoFHRG",
            "02_long_experiment": "1pg3jzNF59YJX_lqVf4dcYI99TyBHcJX_",
            "05_quad_with_dynamics": "1ScfmWiRQ_nGy3Xj5VqRSpzkEJl5BHPQv",
            "06_dynamic_spinning": "1x1f_WfkQIf5AtdRhnWblhkPLur5_5ck0",
            "07_parkland_mound": "1PAywZT8T9TbKy_XJEgWXJkFvr5C6M1pS",
        }[self.seq]

        gt_url = {
            "01_short_experiment": "11VWvHxjitd4ijARD4dJ3WjFuZ_QbInVy",
            "02_long_experiment": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
            "05_quad_with_dynamics": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
            "06_dynamic_spinning": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
            "07_parkland_mound": "1CMcmw9pAT1Mm-Zh-nS87i015CO-xFHwl",
        }[self.seq]

        import gdown

        folder = EVALIO_DATA / NewerCollege2020.name() / self.seq

        print(f"Making folder {folder}...")
        folder.mkdir(parents=True, exist_ok=True)

        print(f"Downloading {self.seq} to {folder}...")
        gdown.download(id=gt_url, output=str(folder / "ground_truth.csv"))
        gdown.download_folder(id=folder_id, output=str(folder))

    # ------------------------- Static Methods ------------------------- #
    @staticmethod
    def name() -> str:
        return "newer_college_2020"

    @staticmethod
    def sequences() -> list[str]:
        return [
            "01_short_experiment",
            "02_long_experiment",
            "05_quad_with_dynamics",
            "06_dynamic_spinning",
            "07_parkland_mound",
        ]

    @staticmethod
    def imu_T_lidar() -> gtsam.Pose3:
        pass

    @staticmethod
    def imu_T_gt() -> gtsam.Pose3:
        pass

    @staticmethod
    def preint_params() -> PreintNoise:
        pass
