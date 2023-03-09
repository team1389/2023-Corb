package frc.util;

import java.util.LinkedList;
import java.util.Queue;

public class SizeLimitedQueue {
    private int size;
    Queue<Double> queue = new LinkedList<>();

    public SizeLimitedQueue(int size) {
        this.size = size;
    }

    public void add(double number) {
        queue.add(number);
        if (queue.size() > size) {
            queue.remove();
        }
    }

    public double getSum() {
        double sum = 0;
        for (double i : queue) {
            sum += i;
        }

        return sum;
    }

    public double length() {
        return size;
    }

    public double getAverage() {
        double sum = 0;
        for (Double num : queue) {
            sum += num;
        }
        return sum / queue.size();
    }

    public double getAverageDerivative() {
        double sum = 0;
        double[] array = new double[queue.size()];
        int i = 0;
        for (Double num : queue) {
            array[i] = num;
            i++;
        }
        for (int j = 0; j < array.length - 1; j++) {
            sum += (array[j + 1] - array[j]) / 1;
        }
        return sum / (array.length);
    }
}