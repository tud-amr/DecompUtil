#ifndef THREADPOOL_H
#define THREADPOOL_H


#include <condition_variable> 
#include <functional> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <thread> 
using namespace std; 
  
// Class that represents a simple thread pool 
class ThreadPool { 
public: 
    // // Constructor to creates a thread pool with given 
    // number of threads 
    ThreadPool(size_t num_threads 
               = thread::hardware_concurrency()) 
    { 
  
        // Creating worker threads 
        for (size_t i = 0; i < num_threads; ++i) { 
            threads_.emplace_back([this] { 
                while (true) { 
                    function<void()> task; 
                    // The reason for putting the below code 
                    // here is to unlock the queue before 
                    // executing the task so that other 
                    // threads can perform enqueue tasks 
                    { 
                        // Locking the queue so that data 
                        // can be shared safely 
                        unique_lock<mutex> lock( 
                            queue_mutex_); 
  
                        // Waiting until there is a task to 
                        // execute or the pool is stopped 
                        cv_.wait(lock, [this] { 
                            return !tasks_.empty() || stop_; 
                        }); 
  
                        // exit the thread in case the pool 
                        // is stopped and there are no tasks 
                        if (stop_ && tasks_.empty()) { 
                            return; 
                        } 
  
                        // Get the next task from the queue 
                        task = move(tasks_.front()); 
                        tasks_.pop(); 
                    } 
  
                    task(); 
                } 
            }); 
        } 
    } 
  
    // Destructor to stop the thread pool 
    ~ThreadPool() 
    { 
        { 
            // Lock the queue to update the stop flag safely 
            unique_lock<mutex> lock(queue_mutex_); 
            stop_ = true; 
        } 
  
        // Notify all threads 
        cv_.notify_all(); 
  
        // Joining all worker threads to ensure they have 
        // completed their tasks 
        for (auto& thread : threads_) { 
            thread.join(); 
        } 
    } 
  
    // Enqueue task for execution by the thread pool 
    void enqueue(function<void()> task) 
    { 
        {
            lock_guard<mutex> lock(task_count_mutex_);
            ++number_of_tasks;  // increase number of pending tasks
        }
        { 
            unique_lock<mutex> lock(queue_mutex_); 
            tasks_.emplace(move(task)); 
        } 
        cv_.notify_one(); 
    } 

    void taskDone() 
    {
        lock_guard<mutex> lock(task_count_mutex_);
        --number_of_tasks;  // decrease number of pending tasks 
        task_cv_.notify_one();  // poke the main thread to wake it up
    }

    void waitUntilFinished()
    {
        unique_lock<mutex> lock(task_count_mutex_);
        task_cv_.wait( lock, [&] () { return (number_of_tasks <= 0); } );
    }
  
private: 
    // Vector to store worker threads 
    vector<thread> threads_; 
  
    // Queue of tasks 
    queue<function<void()> > tasks_; 

    // Number of tasks in queue and being executed
    int number_of_tasks = 0;
  
    // Mutex to synchronize access to shared data 
    mutex queue_mutex_; 

    // Mutex to synchronize access to the number of tasks
    mutex task_count_mutex_;
  
    // Condition variable to signal changes in the state of 
    // the tasks queue 
    condition_variable cv_; 

    // Condition variable to signal changes in state of the number of tasks finished
    condition_variable task_cv_;
  
    // Flag to indicate whether the thread pool should stop 
    // or not 
    bool stop_ = false; 
}; 


#endif // THREADPOOL_H