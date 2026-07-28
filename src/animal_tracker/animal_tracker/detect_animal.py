#!/usr/bin/env python3

import cv2
import numpy as np
import sys
import time

sys.path.append('/usr/local/Ascend/ascend-toolkit/8.0.0/python/site-packages')
import acl

ACL_MEM_MALLOC_HUGE_FIRST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2

CLASS_NAMES = ['elephant', 'tiger', 'monkey', 'bird', 'wolf']


class Net:
    def __init__(self, model_path, device_id=0):
        self.device_id = device_id
        self.model_id = None
        self.model_desc = None
        self.input_dataset = None
        self.output_dataset = None
        self.input_data = []
        self.output_data = []

        acl.init()
        acl.rt.set_device(self.device_id)

        self.model_id, ret = acl.mdl.load_from_file(model_path)
        self.model_desc = acl.mdl.create_desc()
        acl.mdl.get_desc(self.model_desc, self.model_id)

        self.input_dataset, self.input_data = self._prepare_dataset("input")
        self.output_dataset, self.output_data = self._prepare_dataset("output")

    def _prepare_dataset(self, io_type):
        if io_type == "input":
            io_num = acl.mdl.get_num_inputs(self.model_desc)
            get_size = acl.mdl.get_input_size_by_index
        else:
            io_num = acl.mdl.get_num_outputs(self.model_desc)
            get_size = acl.mdl.get_output_size_by_index

        dataset = acl.mdl.create_dataset()
        datas = []
        for i in range(io_num):
            buffer_size = get_size(self.model_desc, i)
            buffer, ret = acl.rt.malloc(buffer_size, ACL_MEM_MALLOC_HUGE_FIRST)
            data_buffer = acl.create_data_buffer(buffer, buffer_size)
            acl.mdl.add_dataset_buffer(dataset, data_buffer)
            datas.append({"buffer": buffer, "data": data_buffer, "size": buffer_size})
        return dataset, datas

    def forward(self, inputs):
        for i in range(len(inputs)):
            bytes_data = inputs[i].tobytes()
            bytes_ptr = acl.util.bytes_to_ptr(bytes_data)
            acl.rt.memcpy(
                self.input_data[i]["buffer"],
                self.input_data[i]["size"],
                bytes_ptr,
                len(bytes_data),
                ACL_MEMCPY_HOST_TO_DEVICE
            )

        ret = acl.mdl.execute(self.model_id, self.input_dataset, self.output_dataset)

        results = []
        for i, item in enumerate(self.output_data):
            host_buffer, ret = acl.rt.malloc_host(item["size"])
            acl.rt.memcpy(
                host_buffer,
                item["size"],
                item["buffer"],
                item["size"],
                ACL_MEMCPY_DEVICE_TO_HOST
            )
            bytes_out = acl.util.ptr_to_bytes(host_buffer, item["size"])
            data = np.frombuffer(bytes_out, dtype=np.float32)
            results.append(data)
            acl.rt.free(host_buffer)

        return results

    def infer(self, img):
        input_tensor = self._preprocess(img)
        output = self.forward([input_tensor])
        return self._postprocess(output[0])

    def _preprocess(self, frame):
        img = cv2.resize(frame, (512, 512))
        img = img[:, :, ::-1]
        img = np.transpose(img, (2, 0, 1))
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)
        return np.ascontiguousarray(img)

    def _postprocess(self, output, conf_thres=0.3, iou_thres=0.45):
        try:
            output = output.reshape(5, 5376)
        except:
            return []

        box = output[:4, :]
        scores = output[4, :]

        mask = scores >= conf_thres
        if not np.any(mask):
            return []

        box = box[:, mask]
        scores = scores[mask]

        cx = box[0, :]
        cy = box[1, :]
        w = box[2, :]
        h = box[3, :]

        x1 = (cx - w / 2) * 512
        y1 = (cy - h / 2) * 512
        x2 = (cx + w / 2) * 512
        y2 = (cy + h / 2) * 512

        dets = np.stack([x1, y1, x2, y2, scores], axis=1)

        if len(dets) == 0:
            return []

        order = dets[:, 4].argsort()[::-1]
        dets = dets[order]

        keep = []
        while len(dets) > 0:
            keep.append(dets[0])
            if len(dets) == 1:
                break
            ious = []
            for d in dets[1:]:
                iou = self._compute_iou(dets[0], d)
                ious.append(iou)
            ious = np.array(ious)
            dets = dets[1:][ious < iou_thres]

        return keep

    def _compute_iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        return inter_area / (area1 + area2 - inter_area + 1e-6)

    def __del__(self):
        for dataset in [self.input_data, self.output_data]:
            while dataset:
                item = dataset.pop()
                acl.destroy_data_buffer(item["data"])
                acl.rt.free(item["buffer"])
        acl.mdl.destroy_dataset(self.input_dataset)
        acl.mdl.destroy_dataset(self.output_dataset)
        acl.mdl.destroy_desc(self.model_desc)
        acl.mdl.unload(self.model_id)
        acl.rt.reset_device(self.device_id)
        acl.finalize()


def find_camera():
    for i in range(4):
        cap = cv2.VideoCapture(i, cv2.CAP_ANY)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cap.release()
                return i
        cap.release()
    return None


def main():
    cam_id = find_camera()
    if cam_id is None:
        print("No camera found!")
        return

    print(f"Using camera /dev/video{cam_id}")

    model_path = "/home/HwHiAiUser/best_animal.om"
    try:
        net = Net(model_path)
        print("Model loaded successfully")
    except Exception as e:
        print(f"Model load failed: {e}")
        return

    cap = cv2.VideoCapture(cam_id, cv2.CAP_ANY)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("Failed to open camera")
        return

    print("Camera ready, press q to quit")

    frame_count = 0
    last_log_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        detections = net.infer(frame)

        frame_count += 1
        current_time = time.time()
        if current_time - last_log_time >= 1.0:
            fps = frame_count / (current_time - last_log_time)
            print(f"[FPS] {fps:.1f}")
            frame_count = 0
            last_log_time = current_time

        if len(detections) > 0:
            for det in detections:
                x1, y1, x2, y2, conf = det
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"animal {conf:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
