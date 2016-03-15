#ifndef __QQUEUE_H__
#define __QQUEUE_H__

#include <stdint.h>

#define QQUEUE64_CAPACITY (64)
#define QQUEUE128_CAPACITY (128)

template <typename T>
class qqueue64 {
  public:
    // Create a queue 
    qqueue64();
    ~qqueue64();

    // Push element to end of queue
    void push(const T e);

    // Pop element off front of queue
    T pop();

    // Number of elements in queue
    uint32_t size() const;

    // Get and element from index
    T operator[](int i) const;

  private:
    T q[QQUEUE64_CAPACITY];

    // Number of elements currently in the queue
    uint32_t s;

    // Head and tail of the queue
    int head;
    int tail;
};

template <typename T>
qqueue64<T>::qqueue64() {
  s = 0;
  head = 0;
  tail = 0;
}

template <typename T>
qqueue64<T>::~qqueue64() {
}

template <typename T>
void qqueue64<T>::push(const T e) {
  if(s >= QQUEUE64_CAPACITY) return;
  q[tail] = e;
  tail = (tail < (QQUEUE64_CAPACITY-1)) ? (tail+1) : 0;
  s++;
}

template <typename T>
T qqueue64<T>::pop() {
  if(s == 0) return T();
  T val = q[head];
  head = (head < (QQUEUE64_CAPACITY-1)) ? (head+1) : 0;
  s--;
  return val;
}

template <typename T>
uint32_t qqueue64<T>::size() const {
  return s;
}

template <typename T>
T qqueue64<T>::operator[](int i) const {
  if(i >= s) return T();
  int idx = head + i;
  return q[(idx < QQUEUE64_CAPACITY) ? idx : (idx - QQUEUE64_CAPACITY)];
}

template <typename T>
class qqueue128 {
  public:
    // Create a queue 
    qqueue128();
    ~qqueue128();

    // Push element to end of queue
    void push(const T e);

    // Pop element off front of queue
    T pop();

    // Number of elements in queue
    uint32_t size() const;

    // Get and element from index
    T operator[](int i) const;

  private:
    T q[QQUEUE128_CAPACITY];

    // Number of elements currently in the queue
    uint32_t s;

    // Head and tail of the queue
    int head;
    int tail;
};

template <typename T>
qqueue128<T>::qqueue128() {
  s = 0;
  head = 0;
  tail = 0;
}

template <typename T>
qqueue128<T>::~qqueue128() {
}

template <typename T>
void qqueue128<T>::push(const T e) {
  if(s >= QQUEUE128_CAPACITY) return;
  q[tail] = e;
  tail = (tail < (QQUEUE128_CAPACITY-1)) ? (tail+1) : 0;
  s++;
}

template <typename T>
T qqueue128<T>::pop() {
  if(s == 0) return T();
  T val = q[head];
  head = (head < (QQUEUE128_CAPACITY-1)) ? (head+1) : 0;
  s--;
  return val;
}

template <typename T>
uint32_t qqueue128<T>::size() const {
  return s;
}

template <typename T>
T qqueue128<T>::operator[](int i) const {
  if(i >= s) return T();
  int idx = head + i;
  return q[(idx < QQUEUE128_CAPACITY) ? idx : (idx - QQUEUE128_CAPACITY)];
}

#endif // __QQUEUE_H__
