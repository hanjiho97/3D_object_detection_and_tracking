import copy
import pickle
import sys
import numpy as np
from skimage import io

from pcdet.ops.roiaware_pool3d import roiaware_pool3d_utils
from pcdet.utils import box_utils, calibration_kitti, common_utils, object3d_kitti
from pcdet.datasets.dataset import DatasetTemplate
from pcdet.models.model_utils import model_nms_utils
import time

class KittiDatasetMM(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, current_index=None):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
            current_index:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.split = self.dataset_cfg.DATA_SPLIT[self.mode] # train or val
        self.root_split_path = self.root_path / ('training' if self.split != 'test' else 'testing') # traing or eval ==> training, else testing

        # split_dir = self.root_path / 'ImageSets' / (self.split + '.txt') # ImageSets의 training.txt, testing.txt, eval.txt 중 선택
        # self.sample_id_list = [x.strip() for x in open(split_dir).readlines()] if split_dir.exists() else None # ImageSets로 id_list 생성
        self.current_index = current_index
        
        self.kitti_infos = self.process_single_scene(num_workers=4, has_label=False, count_inside_pts=False)
        # self.include_kitti_data(self.mode)
       

    def include_kitti_data(self, mode):
        if self.logger is not None:
            self.logger.info('Loading KITTI dataset')
        kitti_infos = []

        for info_path in self.dataset_cfg.INFO_PATH[mode]:
            info_path = self.root_path / info_path
            if not info_path.exists():
                continue
            with open(info_path, 'rb') as f:
                infos = pickle.load(f)
                kitti_infos.extend(infos)

        self.kitti_infos.extend(kitti_infos)

        if self.logger is not None:
            self.logger.info('Total samples for KITTI dataset: %d' % (len(kitti_infos)))


    def set_split(self, split):
        super().__init__(
            dataset_cfg=self.dataset_cfg, class_names=self.class_names, training=self.training, root_path=self.root_path, logger=self.logger
        )
        self.split = split
        self.root_split_path = self.root_path / ('training' if self.split != 'test' else 'testing')

        split_dir = self.root_path / 'ImageSets' / (self.split + '.txt')
        self.sample_id_list = [x.strip() for x in open(split_dir).readlines()] if split_dir.exists() else None

    def get_lidar(self):
        lidar_file = self.root_split_path / 'velodyne' / ('%s.bin' % self.current_index)
        assert lidar_file.exists()
        p = np.fromfile(str(lidar_file), dtype=np.float32).reshape(-1, 4)

        return p

    def get_lidar_mm(self):
        lidar_file = self.root_split_path / self.dataset_cfg.MM_PATH / ('%s.npy' % self.current_index)
        assert lidar_file.exists()
        return np.load(lidar_file).astype(np.float32)

    def get_image_shape(self):
        img_file = self.root_split_path / 'image_2' / ('%s.png' % self.current_index)
        assert img_file.exists()
        return np.array(io.imread(img_file).shape[:2], dtype=np.int32)

    def get_image(self):
        img_file = self.root_split_path / 'image_2' / ('%s.png' % self.current_index)
        assert img_file.exists()
        return np.array(io.imread(img_file))

    def get_label(self):
        label_file = self.root_split_path / 'label_2' / ('%s.txt' % self.current_index)
        assert label_file.exists()
        return object3d_kitti.get_objects_from_label(label_file)

    def get_calib(self):
        calib_file = self.root_split_path / 'calib' / ('%s.txt' % self.current_index)
        assert calib_file.exists()
        return calibration_kitti.Calibration(calib_file)

    def get_road_plane(self):
        plane_file = self.root_split_path / 'planes' / ('%s.txt' % self.current_index)
        if not plane_file.exists():
            return None

        with open(plane_file, 'r') as f:
            lines = f.readlines()
        lines = [float(i) for i in lines[3].split()]
        plane = np.asarray(lines)

        # Ensure normal is always facing up, this is in the rectified camera coordinate
        if plane[1] > 0:
            plane = -plane

        norm = np.linalg.norm(plane[0:3])
        plane = plane / norm
        return plane

    @staticmethod
    def get_fov_flag(pts_rect, img_shape, calib):
        """
        Args:
            pts_rect:
            img_shape:
            calib:

        Returns:

        """
        pts_img, pts_rect_depth = calib.rect_to_img(pts_rect)
        val_flag_1 = np.logical_and(pts_img[:, 0] >= 0, pts_img[:, 0] < img_shape[1])
        val_flag_2 = np.logical_and(pts_img[:, 1] >= 0, pts_img[:, 1] < img_shape[0])
        val_flag_merge = np.logical_and(val_flag_1, val_flag_2)
        pts_valid_flag = np.logical_and(val_flag_merge, pts_rect_depth >= 0)

        return pts_valid_flag

    
    import concurrent.futures as futures

    def process_single_scene(self, num_workers=4, has_label=True, count_inside_pts=True):
        print('%s current_index: %s' % (self.split, self.current_index))
        info = {}
        pc_info = {'num_features': 4, 'lidar_idx': self.current_index}
        info['point_cloud'] = pc_info

        image_info = {'image_idx': self.current_index, 'image_shape': self.get_image_shape()}
        info['image'] = image_info
        calib = self.get_calib()

        P2 = np.concatenate([calib.P2, np.array([[0., 0., 0., 1.]])], axis=0)
        R0_4x4 = np.zeros([4, 4], dtype=calib.R0.dtype)
        R0_4x4[3, 3] = 1.
        R0_4x4[:3, :3] = calib.R0
        V2C_4x4 = np.concatenate([calib.V2C, np.array([[0., 0., 0., 1.]])], axis=0)
        calib_info = {'P2': P2, 'R0_rect': R0_4x4, 'Tr_velo_to_cam': V2C_4x4}

        info['calib'] = calib_info
        tmp_list = []
        tmp_list.append(info)

        return tmp_list

    #staticmethod
    def generate_prediction_dicts(self,batch_dict, pred_dicts, class_names, output_path=None):
        """
        Args:
            batch_dict:
                frame_id:
            pred_dicts: list of pred_dicts
                pred_boxes: (N, 7), Tensor
                pred_scores: (N), Tensor
                pred_labels: (N), Tensor
            class_names:
            output_path:

        Returns:

        """
        def get_template_prediction(num_samples):
            ret_dict = {
                'name': np.zeros(num_samples), 'truncated': np.zeros(num_samples),
                'occluded': np.zeros(num_samples), 'alpha': np.zeros(num_samples),
                'bbox': np.zeros([num_samples, 4]), 'dimensions': np.zeros([num_samples, 3]),
                'location': np.zeros([num_samples, 3]), 'rotation_y': np.zeros(num_samples),
                'score': np.zeros(num_samples), 'boxes_lidar': np.zeros([num_samples, 7])
            }
            return ret_dict

        def generate_single_sample_dict(batch_index, box_dict):
            pred_scores = box_dict['pred_scores'].cpu().numpy()
            pred_boxes = box_dict['pred_boxes'].cpu().numpy()
            pred_labels = box_dict['pred_labels'].cpu().numpy()

            if 'WBF' in box_dict:
                pred_labels,pred_scores,pred_boxes = model_nms_utils.compute_WBF(pred_labels,
                                                                                 pred_scores,
                                                                                 pred_boxes,
                                                                                 iou_thresh=box_dict['IoU'],
                                                                                 retain_low=box_dict['RL'],
                                                                                 score_thresh=box_dict['SCORE_THRESH'])

            pred_dict = get_template_prediction(pred_scores.shape[0])
            if pred_scores.shape[0] == 0:
                return pred_dict

            calib = batch_dict['calib'][batch_index]
            image_shape = batch_dict['image_shape'][batch_index]
            pred_boxes_camera = box_utils.boxes3d_lidar_to_kitti_camera(pred_boxes, calib)
            pred_boxes_img = box_utils.boxes3d_kitti_camera_to_imageboxes(
                pred_boxes_camera, calib, image_shape=image_shape
            )

            pred_dict['name'] = np.array(class_names)[pred_labels - 1]
            pred_dict['alpha'] = -np.arctan2(-pred_boxes[:, 1], pred_boxes[:, 0]) + pred_boxes_camera[:, 6]
            pred_dict['bbox'] = pred_boxes_img
            height = pred_dict['bbox'][:, 3] - pred_dict['bbox'][:, 1]
            height_mask = height<25
            pred_dict['bbox'][height_mask, 3] +=2
            pred_dict['dimensions'] = pred_boxes_camera[:, 3:6]
            pred_dict['location'] = pred_boxes_camera[:, 0:3]
            pred_dict['rotation_y'] = pred_boxes_camera[:, 6]
            pred_dict['score'] = pred_scores
            pred_dict['boxes_lidar'] = pred_boxes

            return pred_dict

        annos = []
        for index, box_dict in enumerate(pred_dicts):
            frame_id = batch_dict['frame_id'][index]

            single_pred_dict = generate_single_sample_dict(index, box_dict)



            single_pred_dict['frame_id'] = frame_id
            annos.append(single_pred_dict)

            
            if output_path is not None:
                cur_det_file = '/root/VirConv/data/output/' + ('%s.txt' % frame_id)
                total_file = '/root/VirConv/data/output/frame.txt'
                 
                with open(cur_det_file, 'w') as f:
                    bbox = single_pred_dict['bbox']
                    loc = single_pred_dict['location']
                    dims = single_pred_dict['dimensions']  # lhw -> hwl

                    for idx in range(len(bbox)):
                        print('%s -1 -1 %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f'
                              % (single_pred_dict['name'][idx], single_pred_dict['alpha'][idx],
                                 bbox[idx][0], bbox[idx][1], bbox[idx][2], bbox[idx][3],
                                 dims[idx][1], dims[idx][2], dims[idx][0], loc[idx][0],
                                 loc[idx][1], loc[idx][2], single_pred_dict['rotation_y'][idx],
                                 single_pred_dict['score'][idx]), file=f)

                with open(total_file, 'w') as t:
                    print('%s' %frame_id, file=t)
                    
        return annos

    def evaluation(self, det_annos, class_names, **kwargs):
        if 'annos' not in self.kitti_infos[0].keys():
            return None, {}

        from .kitti_object_eval_python import eval as kitti_eval

        eval_det_annos = copy.deepcopy(det_annos)
        eval_gt_annos = [copy.deepcopy(info['annos']) for info in self.kitti_infos]
        ap_result_str, ap_dict = kitti_eval.get_official_eval_result(eval_gt_annos, eval_det_annos, class_names)

        return ap_result_str, ap_dict

    def __len__(self):
        if self._merge_all_iters_to_one_epoch:
            return len(self.kitti_infos) * self.total_epochs

        return len(self.kitti_infos)

    def __getitem__(self, index):
        # index = 4
        
        if self._merge_all_iters_to_one_epoch:
            index = index % len(self.kitti_infos)

        info = copy.deepcopy(self.kitti_infos[index])

        sample_idx = info['point_cloud']['lidar_idx']

        
        points = self.get_lidar_mm()
        
        calib = self.get_calib()

        img_shape = info['image']['image_shape']
        if self.dataset_cfg.FOV_POINTS_ONLY:
            pts_rect = calib.lidar_to_rect(points[:, 0:3])
            fov_flag = self.get_fov_flag(pts_rect, img_shape, calib)
            points = points[fov_flag]

        input_dict = {
            'points': points,
            'frame_id': sample_idx,
            'calib': calib,
        }
        input_dict.update({
                'mm': np.ones(shape=(1, 1))
            })

        if 'annos' in info:
            annos = info['annos']
            annos = common_utils.drop_info_with_name(annos, name='DontCare')
            loc, dims, rots = annos['location'], annos['dimensions'], annos['rotation_y']
            gt_names = annos['name']

            if (self.dataset_cfg.get('USE_VAN', None) is True) and (self.training is True):
                gt_names = np.array(['Car' if gt_names[i]=='Van' else gt_names[i] for i in range(len(gt_names))])

            gt_boxes_camera = np.concatenate([loc, dims, rots[..., np.newaxis]], axis=1).astype(np.float32)
            gt_boxes_lidar = box_utils.boxes3d_kitti_camera_to_lidar(gt_boxes_camera, calib)
            if self.training and 'num_points_in_gt' in annos:
                nmask = annos['num_points_in_gt']>0
                annos['num_points_in_gt'] = annos['num_points_in_gt'][nmask]
                gt_names = gt_names[nmask]
                gt_boxes_lidar = gt_boxes_lidar[nmask]

            input_dict.update({
                'gt_names': gt_names,
                'gt_boxes': gt_boxes_lidar
            })


            road_plane = self.get_road_plane(sample_idx)
            if road_plane is not None:
                input_dict['road_plane'] = road_plane


        
        data_dict = self.prepare_data(data_dict=input_dict)
        data_dict['image_shape'] = [img_shape]
        data_dict['calib'] = [calib]
        data_dict['frame_id'] = [data_dict['frame_id']]
        data_dict['use_lead_xyz'] = [data_dict['use_lead_xyz']]

        
        data_dict['points'] = np.insert(data_dict['points'], 0, values=0, axis=1)
        data_dict['voxel_coords'] = np.insert(data_dict['voxel_coords'], 0, values=0, axis=1)
        data_dict['batch_size'] = 1
        
        
        return data_dict


if __name__ == '__main__':
    import sys
    if sys.argv.__len__() > 1 and sys.argv[1] == 'create_kitti_infos':
        import yaml
        from pathlib import Path
        from easydict import EasyDict
        dataset_cfg = EasyDict(yaml.safe_load(open(sys.argv[2])))
        ROOT_DIR = (Path(__file__).resolve().parent / '../../../').resolve()
