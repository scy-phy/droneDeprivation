## README

##### tldr: Plots and Evaluation metrics
- Move to `logs` directory
- Run `$ python3 plot_evaluation.py`
- Shows data and plot for 4 different cases

-----

##### More information
- Directory structure
```
.
├── bin2csv.sh                       -> copied from data_analysis dir
├── logs
│   ├── plot_evaluation.py           -> Gives summary of experiments
│   └── *EVL6.csv                    -> csv filtered from *.bin
├── mavlogdump_modified.py           -> copied from data_analysis dir
├── README.md
└── *.bin                            -> original logs from Kakute
```
- Experiment information
```
.
├── test01_no_attack_long.bin          -> no attack for 5:20 minutes
├── test02_no_attack_short.bin         -> no attack for 2:00 min
├── test03_suspend_once.bin            -> Suspend attack starts at 0:28
                                       -> Never turned off
├── test04_suspend_repeat.bin          -> Suspend attack on-off cycle
├── test05_freq_once.bin               -> Frequency attack starts at 0:28
                                       -> Never turned off
├── test06_freq_repeat.bin             -> Frequency attack on-off cycle
└── test07_freq_once_100hz.bin         -> Modified Frequency to 100 Hz
```
- Log files `*.bin` are generated with `EVL6` logging added to ArduPilot code
```python
AP::logger().Write("EVL6", "TimeUS,SPI_txn,SPI_len,GT", "QIII", AP_HAL::micros64(), hal.util->persistent_spi_data.spi_txn_count, hal.util->persistent_spi_data.spi_len_count, ground_truth);
```

##### Using different `bin` file
- Extract `EVL6` related logs using `$ ./bin2csv.sh <file_name.bin>`. This generates csv entry in `logs/`
- To plot `<file_name.csv>` move to `logs` directory and use `$ python3 plot_evaluation.py <file_name.csv>`
- Make changes to `plot_evaluation.py` for more flexibility

##### Information generated
- `plot_evaluation.csv` provides:
	1. 5 Plots: SPI packets/sec, SPI packet length/sec, Ground Truth, Relative Log messages/sec, Attack detected?
	2. Prints: Upper and Lower SPI packet length/sec limit from `test01_no_attack_long_EVL6.csv`, Confusion matrix, More statistics.