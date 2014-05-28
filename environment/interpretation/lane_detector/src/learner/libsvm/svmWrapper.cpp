#include <libsvm/svmWrapper.hpp>

/*************************************************************/
/***************  Class Member Functions  ********************/

/*************************************************************/

int SVM::init(int inputDimension) {
    max_nr_attr = inputDimension;

    x = (struct svm_node *) malloc((inputDimension + 1) * sizeof (struct svm_node));

    return 0;
}

SVM::SVM() {
    storeOutput = 0;
}

SVM::~SVM() {
    svm_free_and_destroy_model(&model);
    free(x);
    inputFile.close();
    outputFile.close();
}

int SVM::parseInput(int argc, char *argv[]) {
    if (argc < 3) {
        return -1;
    }

    inputFile.open(argv[1]);
    if (!inputFile.is_open()) {
        std::cout << " The input file (" << argv[1] << ") does not exist! " << std::endl;
        return -1;
    }

    int load_err = loadModel(std::string(argv[2]));
    if (load_err != 0) {
        return load_err;
    }

    if (argc == 4) {
        outputFile.open(argv[3]);
        if (!outputFile.is_open()) {
            std::cout << " The output file (" << argv[3] << ") does not exist! " << std::endl;
            std::cout << " The output will not be stored! " << std::endl;
            storeOutput = 0;
            return -1;
        } else {
            storeOutput = 1;
        }
    } else {
        std::cout << " No output file specified! " << std::endl;
        std::cout << " The output will not be stored! " << std::endl;
        storeOutput = 0;
    }

    return 0;
}

int SVM::loadModel(std::string modelFile) {
    if ((model = svm_load_model(modelFile.c_str())) == 0) {
        fprintf(stderr, "can't open model file %s\n", modelFile.c_str());
        return -1;
    }
    return 0;
}

int SVM::predictFromFile() {
    int correct = 0;
    int total = 0;
    double error = 0;
    double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;
    int svm_type = svm_get_svm_type(model);

    std::string line;
    std::vector<double> dataEntries;
    int lineNumber = 0;
    while (!inputFile.eof()) {
        int target = 99, prediction = 99;
        if (getline(inputFile, line) == NULL) {
            break;
        }
        lineNumber++;
        dataEntries = split(line, '\t');
        if (dataEntries.size() != (max_nr_attr + 1)) {
            printf("dataEntries.size() ---------------- %d - Required: %d\t\n", (int) dataEntries.size(), (max_nr_attr + 1));
            std::cout << "Incorrect File Format at line " << lineNumber << "\n";
            return -1;
        }

        target = (int) dataEntries[max_nr_attr];
        predict(dataEntries, prediction);

        if (storeOutput == 1) {
            outputFile << target << "\t" << prediction << std::endl;
        }

        if (target == prediction) {
            ++correct;
        }
        error += (prediction - target)*(prediction - target);
        sump += prediction;
        sumt += target;
        sumpp += prediction*prediction;
        sumtt += target*target;
        sumpt += prediction*target;
        ++total;
    }

    if (svm_type == NU_SVR || svm_type == EPSILON_SVR) {
        printf("Mean squared error = %g (regression)\n", error / total);
        printf("Squared correlation coefficient = %g (regression)\n", ((total * sumpt - sump * sumt)*(total * sumpt - sump * sumt)) / ((total * sumpp - sump * sump)*(total * sumtt - sumt * sumt)));
    } else {
        printf("Accuracy = %g%% (%d/%d) (classification)\n", (double) correct / total * 100, correct, total);
    }
    return 0;
}

int SVM::predict(std::vector<double> _input_, int &_output_) {
    int i;
    if (_input_.size() < max_nr_attr) {
        printf("Wrong Input Size.\n");
    }
    for (i = 0; i < max_nr_attr; i++) {
        x[i].index = i;
        x[i].value = _input_[i];
    }
    x[max_nr_attr].index = -1;
    _output_ = svm_predict(model, x);
    return 0;
}

std::vector<double> SVM::split(std::string work, char delim, int rep) {
    std::vector<double> d_;
    if (!d_.empty()) {
        d_.clear(); // empty vector if necessary
    }
    std::string buf = "";
    int i = 0;
    while (i < work.length()) {
        if (work[i] != delim)
            buf += work[i];
        else if (rep == 1) {
            d_.push_back((double) atof(buf.c_str()));
            buf = "";
        } else if (buf.length() > 0) {
            d_.push_back((double) atof(buf.c_str()));
            buf = "";
        }
        i++;
    }
    if (!buf.empty())
        d_.push_back((double) atof(buf.c_str()));
    return d_;
}

int SVM::predict(cv::Mat _input_, int &_output_) {
    int i = 0;
    if (_input_.cols * _input_.rows * 3 != max_nr_attr) {
        printf("Wrong Input Size.\n");
    }
    for (int r = 0; r < _input_.rows; r++) {
        for (int c = 0; c < _input_.cols; c++) {
            x[i].index = i;
            x[i].value = (double) _input_.at<cv::Vec3b>(r, c)[0];
            i++;
            x[i].index = i;
            x[i].value = (double) _input_.at<cv::Vec3b>(r, c)[1];
            i++;
            x[i].index = i;
  //          x[i].value = (double) _input_.at<cv::Vec3b>(r, c)[2];
    //        i++;
        }
    }
    x[max_nr_attr].index = -1;
    _output_ = svm_predict(model, x);
    return 0;
}

int SVM::predictKernelWise(cv::Mat &_input_, int kernel_size) {
    cv::Mat tempImage = _input_.clone();
    cv::Mat imgRoi;
    int result = 99;
    int start_row, start_col;

    for (int i = 0; i < _input_.rows; i += kernel_size) {
        for (int j = 0; j < _input_.cols; j += kernel_size) {
            if ((_input_.rows - i - 1) > kernel_size && (_input_.cols - j - 1) > kernel_size) {
                start_col = j;
                start_row = i;
            } else if ((_input_.rows - i - 1) > kernel_size) {
                start_col = _input_.cols - kernel_size - 1;
                start_row = i;
            } else if ((_input_.cols - j - 1) > kernel_size) {
                start_col = j;
                start_row = _input_.rows - kernel_size - 1;
            } else {
                start_col = _input_.cols - kernel_size - 1;
                start_row = _input_.rows - kernel_size - 1;
            }

            imgRoi = tempImage(cv::Rect(start_col, start_row, kernel_size, kernel_size));
            predict(imgRoi, result);
            if (result == 1) {
                cv::rectangle(_input_, cv::Point(start_col, start_row), cv::Point(start_col + kernel_size, start_row + kernel_size), cv::Scalar(0, 0, 0), CV_FILLED);
            }
        }
    }
    return 0;
}
