import sys
import matplotlib.pyplot as plt
from math import *
import numpy as np

lower_limit = 0
upper_limit = 0


def calculate_eval_boundary(file, cal_t_start, cal_t_end):
    gt_ts = []
    gt_spi_len = []
    gt_spi_len_freq = []
    with open(file, 'r') as reader:
        for index, line in enumerate(reader):
            if index == 0:
                pass
            else:
                line = line.rstrip()
                split_text = line.split(",")
                ts = float(split_text[0])
                second_value = floor(ts)
                base_ts = int(second_value)
                if base_ts < cal_t_start or base_ts > cal_t_end:
                    pass
                else:
                    gt_ts.append(ts)
                    gt_spi_len.append(float(split_text[3]))

    for i in range(len(gt_ts) - 1):
        delta_t = float(gt_ts[i + 1]) - float(gt_ts[i])
        delta_spi_len = int(gt_spi_len[i + 1]) - int(gt_spi_len[i])
        gt_spi_len_freq.append(delta_spi_len / delta_t)

    global lower_limit
    global upper_limit
    lower_limit = np.quantile(gt_spi_len_freq, 0.000)
    upper_limit = np.quantile(gt_spi_len_freq, 1.0)
    print(f"Lower limit: {lower_limit}")
    print(f"Upper limit: {upper_limit}")
    print("--------------------------------------")


def evaluate_csv(file, cal_t_start, cal_t_end, plot_title):
    time_stamps = []
    spi_txn = []
    spi_len = []
    gt = []
    spi_txn_frequency = []
    spi_len_frequency = []

    reading_sec_value = []
    readings_per_sec = []

    # delta_list = []
    with open(file, 'r') as reader:
        for index, line in enumerate(reader):
            if index == 0:
                pass
            else:
                line = line.rstrip()
                split_text = line.split(",")
                ts = float(split_text[0])
                second_value = floor(ts)
                base_ts = int(second_value)
                if base_ts < cal_t_start or base_ts > cal_t_end:
                    pass
                # if False:
                #     pass
                else:
                    time_stamps.append(ts)
                    if len(reading_sec_value) == 0:
                        reading_sec_value.append(base_ts)
                        readings_per_sec.append(1)
                    elif reading_sec_value[-1] == base_ts:
                        readings_per_sec[-1] += 1
                    elif reading_sec_value[-1] < base_ts:
                        reading_sec_value.append(base_ts)
                        readings_per_sec.append(1)
                    spi_txn.append(float(split_text[2]))
                    spi_len.append(float(split_text[3]))
                    gt.append(float(split_text[4]))

    for i in range(len(time_stamps) - 1):
        delta_t = float(time_stamps[i + 1]) - float(time_stamps[i])
        delta_spi = int(spi_txn[i + 1]) - int(spi_txn[i])
        delta_spi_len = int(spi_len[i + 1]) - int(spi_len[i])
        spi_txn_frequency.append(delta_spi / delta_t)
        spi_len_frequency.append(delta_spi_len / delta_t)
        # delta_list.append(1/(float(time_stamps[i+1]) - float(time_stamps[i])))

    # ratio_list = []
    # for i in range(len(delta_list)):
    #     ratio_list.append(spi_len_frequency[i]/delta_list[i])

    global lower_limit
    global upper_limit
    upper_alarms = np.array(spi_len_frequency) > upper_limit
    lower_alarm = np.array(spi_len_frequency) < lower_limit
    detections = [upper_alarms | lower_alarm] * 1

    # Confusion matrix calculations
    # offset gt by one since we are using differences
    ground_truths = gt[1:]
    detect = detections[0]
    tp = 0
    tn = 0
    fn = 0
    fp = 0
    for i in range(len(gt[1:])):
        if int(ground_truths[i]) == 100 and detect[i] == True:
            # True Positive
            tp += 1
        elif int(ground_truths[i]) == 0 and detect[i] == False:
            # True Negative
            tn += 1
        elif int(ground_truths[i]) == 100 and detect[i] == False:
            # False Negative
            fn += 1
        elif int(ground_truths[i]) == 0 and detect[i] == True:
            # False Positive
            fp += 1

    print("Confusion Matrix:")
    print(f'TP: {tp:>10}, FN: {fn:>10}')
    print(f'FP: {fp:>10}, TN: {tn:>10}')
    accuracy = (tp + tn) / (tp + tn + fp + fn)
    print(f"Accuracy: {round(accuracy, 6)}")
    # print(f"Recall: {(tp)/(tp + fn)}")
    # print(f"Precision: {(tp)/(tp + fp)}")

    # use sharex to allow x-axis sync in all 5 plots
    fig, axs = plt.subplots(5, sharex=True)
    fig.suptitle(plot_title)
    # offset by 1 time_stamp
    axs[0].plot(time_stamps[1:], spi_txn_frequency)
    axs[1].plot(time_stamps[1:], spi_len_frequency)
    axs[2].plot(time_stamps[1:], gt[1:])
    axs[3].bar(reading_sec_value, readings_per_sec)
    axs[4].plot(time_stamps[1:], detections[0])
    # axs[4].plot(time_stamps[1:], delta_list)
    # axs[5].plot(time_stamps[1:], ratio_list)

    axs.flat[0].set(xlabel='Time (seconds)', ylabel='SPI packet/sec')
    axs.flat[1].set(xlabel='Time (seconds)', ylabel='SPI packet length/sec')
    axs.flat[2].set(xlabel='Time (seconds)', ylabel='Ground Truth')
    axs.flat[3].set(xlabel='Time (seconds)', ylabel='Relative log msg/sec')
    axs.flat[4].set(xlabel='Time (seconds)', ylabel='Attack Detected?')
    print("Close current matplotlib window to continue.")
    plt.show()
    print("--------------------------------------")


def main():
    if len(sys.argv) != 1 and len(sys.argv) != 2:
        print(f"Default Usage: python3 {sys.argv[0]}")
        print(f"Using custom csv file: python3 {sys.argv[0]} <*EVL6.csv file>")
        return 0

    # Can change integer values in function: calculate_eval_boundary, evaluate_csv to change x-axis time range
    print("Calculating MMIO access boundary (300 sec measurement) for evaluation:")
    calculate_eval_boundary("test01_no_attack_long_EVL6.csv", 5, 304)

    if len(sys.argv) == 1:
        print("Evaluation 1: No attack: 60 seconds")
        evaluate_csv("test02_no_attack_short_EVL6.csv", 5, 64, "No attack: 60 seconds")
        print("Evaluation 2: Suspend Attack: 60 seconds")
        evaluate_csv("test03_suspend_once_EVL6.csv", 35, 94, "Suspend Attack: 60 seconds")
        print("Evaluation 3: Frequency attack (default values): 60 seconds")
        evaluate_csv("test05_freq_once_EVL6.csv", 35, 94, "Frequency attack (default values): 60 seconds")
        print("Evaluation 4: Frequency attack (100 Hz): 60 seconds")
        evaluate_csv("test07_freq_once_100hz_EVL6.csv", 35, 94, "Frequency attack (100 Hz): 60 seconds")
    elif len(sys.argv) == 2:
        print("Custom evaluation")
        # 1000000 is a placeholder number for large value
        evaluate_csv(f"{sys.argv[1]}", 0, 1000000, "Custom evaluation")


if __name__ == '__main__':
    main()
