from re import S
import sys
import json
import xlsxwriter

from os import listdir
from os.path import isfile, join

KEY2LOCATION = {
    "CPU": ["CPU"],
    "VirtualMemory": ["VirtualMemory"],
    "PhysicalMemory": ["PhysicalMemory"],
    "AvgFPS": ["AvgFPS"],
    "ORBExtraction": ["Threads", "Tracking", "Subprocess", "ORBExtraction"],
    "Track": ["Threads", "Tracking", "Subprocess", "Track"],
    "ProcessNewKeyFrame": ["Threads", "LocalMapping", "Subprocess", "ProcessNewKeyFrame"],
    "MapPointCulling": ["Threads", "LocalMapping", "Subprocess", "MapPointCulling"],
    "CreateNewMapPoints": ["Threads", "LocalMapping", "Subprocess", "CreateNewMapPoints"],
    "SearchInNeighbors": ["Threads", "LocalMapping", "Subprocess", "SearchInNeighbors"],
    "LocalBundleAdjustment": ["Threads", "LocalMapping", "Subprocess", "LocalBundleAdjustment"],
    "KeyFrameCulling": ["Threads", "LocalMapping", "Subprocess", "KeyFrameCulling"],
    "DetectLoop": ["Threads", "LoopClosing", "Subprocess", "DetectLoop"],
    "ComputeSim3": ["Threads", "LoopClosing", "Subprocess", "ComputeSim3"],
    "DetectLoop&ComputeSim3": ["Threads", "LoopClosing", "Subprocess", "DetectLoop&ComputeSim3"],
    "SearchAndFuse": ["Threads", "LoopClosing", "Subprocess", "SearchAndFuse"],
    "OptimizeEssentialGraph": ["Threads", "LoopClosing", "Subprocess", "OptimizeEssentialGraph"],
    "GlobalBundleAdjustment": ["Threads", "BundleAdjustment", "Subprocess", "GlobalBundleAdjustment"],
    "MapUpdate": ["Threads", "BundleAdjustment", "Subprocess", "MapUpdate"],
}

KEY2LOCATION_TIME = {
    "Tracking": ["Threads", "Tracking", "ProcessTime"],
    "LoopClosing": ["Threads", "LoopClosing", "ProcessTime"],
    "LocalMapping": ["Threads", "LocalMapping", "ProcessTime"],
    "BundleAdjustment": ["Threads", "BundleAdjustment", "ProcessTime"],
}

KEY2ROW = {
    "ORBExtraction": 0,
    "Track": 1,
    "ProcessNewKeyFrame": 2,
    "MapPointCulling": 3,
    "CreateNewMapPoints": 4,
    "SearchInNeighbors": 5,
    "LocalBundleAdjustment": 6,
    "KeyFrameCulling": 7,
    "DetectLoop": 8,
    "ComputeSim3": 9,
    "SearchAndFuse": 10,
    "OptimizeEssentialGraph": 11,
    "GlobalBundleAdjustment": 12,
    "MapUpdate": 13,
    "AvgFPS": 14,
    "CPU": 15,
    "VirtualMemory": 16,
    "PhysicalMemory": 17,
}

TIME_ATTRIBUTES = ["ORBExtraction", "Track", "ProcessNewKeyFrame", "MapPointCulling", "CreateNewMapPoints",
    "SearchInNeighbors", "LocalBundleAdjustment", "KeyFrameCulling", "DetectLoop", "ComputeSim3", "SearchAndFuse",
    "OptimizeEssentialGraph", "GlobalBundleAdjustment", "MapUpdate"]

SIZE_ATTRIBUTES = ["VirtualMemory", "PhysicalMemory"]

PERFORMANCE_ATTRIBUTES = ["CPU", "VirtualMemory", "PhysicalMemory"]

class Metric:
    def __init__(self, min_val, max_val, avg_val, std_val, sum_val, count_val):
        self.min_val = min_val
        self.max_val = max_val
        self.avg_val = avg_val
        self.std_val = std_val
        self.sum_val = sum_val
        self.count_val = count_val

def read_json(json_content, location) -> Metric:
    for key in location:
        if key in json_content:
            json_content = json_content[key]
        else:
            return None
    return json_content

def simplify(content):
    return content.rstrip('0').rstrip('.')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print('Usage: path_to_summary_jsons')
        exit(1)

    # Create an new Excel file and add a worksheet.
    workbook = xlsxwriter.Workbook('summary.xlsx')
    worksheet = workbook.add_worksheet()

    merge_format = workbook.add_format({
        'align': 'center',
        'valign': 'vcenter'})

    worksheet.set_column(0, 1, 23)

    worksheet.write(0, 0, 'Attribute', merge_format)
    worksheet.write(0, 1, 'SubAttribute', merge_format)

    worksheet.merge_range(1, 0, 1, 1, 'Dataset', merge_format)
    worksheet.merge_range(2, 0, 9, 0, 'Tracking', merge_format)
    worksheet.merge_range(10, 0, 33, 0, 'LocalMapping', merge_format)
    worksheet.merge_range(34, 0, 49, 0, 'LoopClosing', merge_format)
    worksheet.merge_range(50, 0, 57, 0, 'BundleAdjustment', merge_format)
    worksheet.merge_range(58, 0, 70, 0, 'Performance', merge_format)

    start = 2
    for i in range(len(TIME_ATTRIBUTES)):
        worksheet.merge_range(start + i, 1, start + i + 3, 1, TIME_ATTRIBUTES[i])
        start += 3

    worksheet.write(58, 1, 'AvgFPS')
    start = 59
    for i in range(len(PERFORMANCE_ATTRIBUTES)):
        start += 3
    worksheet.merge_range(59, 1, 62, 1, 'CPU (%)')
    worksheet.merge_range(63, 1, 66, 1, 'VirtualMemory (mb)')
    worksheet.merge_range(67, 1, 70, 1, 'PhysicalMemory (mb)')

    for i in range(len(TIME_ATTRIBUTES)):
        worksheet.write(2 + i * 4, 2, "avg")
        worksheet.write(3 + i * 4, 2, "std")
        worksheet.write(4 + i * 4, 2, "count")
        worksheet.write(5 + i * 4, 2, "sum")

    worksheet.write(58, 2, "avg")

    for i in range(3):
        worksheet.write(59 + i * 4, 2, "avg")
        worksheet.write(60 + i * 4, 2, "std")
        worksheet.write(61 + i * 4, 2, "count")
        worksheet.write(62 + i * 4, 2, "sum")

    json_files = [join(sys.argv[1], f) for f in listdir(sys.argv[1]) if f.endswith('.json') and isfile(join(sys.argv[1], f))]

    file_count = 0
    for json_file in json_files:
        with open(json_file) as f:
            content = json.load(f)
            if "SystemName" not in content:
                continue
            infos = {key: read_json(content, KEY2LOCATION[key]) for key in KEY2LOCATION if read_json(content, KEY2LOCATION[key]) is not None}
            worksheet.write(0, file_count + 3, content['SystemName'])
            worksheet.write(1, file_count + 3, json_file[:-5].split("_")[-1])
            
            start_index = 2
            for info in infos:
                if info in KEY2ROW.keys():
                    if type(infos[info]) == float:
                        worksheet.write(start_index + KEY2ROW[info] * 4, file_count + 3, "{:.4f}".format(infos[info]))
                    else:
                        avg_time = abs(infos[info]['avg']) / (10.0e6 if info in TIME_ATTRIBUTES else 1.0)
                        avg_time = abs(avg_time) / (10.0e3 if info in SIZE_ATTRIBUTES else 1.0)
                        avg_time = simplify("{:.8f}".format(avg_time))
                        std_time = abs(infos[info]['std']) / (10.0e6 if info in TIME_ATTRIBUTES else 1.0)
                        std_time = abs(std_time) / (10.0e3 if info in SIZE_ATTRIBUTES else 1.0)
                        std_time = simplify("{:.8f}".format(std_time))
                        count = str(infos[info]['count'])
                        sum_val = abs(infos[info]['sum']) / (10.0e6 if info in TIME_ATTRIBUTES else 1.0)
                        sum_val = abs(sum_val) / (10.0e3 if info in SIZE_ATTRIBUTES else 1.0)
                        sum_val = simplify("{:.8f}".format(sum_val))
                        offset = -3 if info in PERFORMANCE_ATTRIBUTES else 0
                        worksheet.write(start_index + KEY2ROW[info] * 4 + offset, file_count + 3, avg_time)
                        worksheet.write(start_index + KEY2ROW[info] * 4 + 1 + offset, file_count + 3, std_time)
                        worksheet.write(start_index + KEY2ROW[info] * 4 + 2 + offset, file_count + 3, count)
                        worksheet.write(start_index + KEY2ROW[info] * 4 + 3 + offset, file_count + 3, sum_val)
                else:
                    avg_time = simplify("{:.8f}".format(abs(infos[info]['avg']) / 10.0e6))
                    std_time = simplify("{:.8f}".format(abs(infos[info]['std']) / 10.0e6))
                    sum_time = simplify("{:.8f}".format(abs(infos[info]['sum']) / 10.0e6))
                    count = str(infos[info]['count'])
                    worksheet.merge_range(34, file_count + 3, 35, file_count + 3, avg_time, workbook.add_format({'valign': 'vcenter'}))
                    worksheet.merge_range(36, file_count + 3, 37, file_count + 3, std_time, workbook.add_format({'valign': 'vcenter'}))
                    worksheet.merge_range(38, file_count + 3, 39, file_count + 3, count, workbook.add_format({'valign': 'vcenter'}))
                    worksheet.merge_range(40, file_count + 3, 41, file_count + 3, sum_time, workbook.add_format({'valign': 'vcenter'}))
            file_count += 1
    worksheet.set_column(3, 3 + file_count, 25)
    workbook.close()
