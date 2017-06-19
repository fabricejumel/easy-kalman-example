#ifndef KALMAN_FILTER_EXAMPLES_KALMAN_FILTER_H
#define KALMAN_FILTER_EXAMPLES_KALMAN_FILTER_H

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

    // state vector
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transition matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;

    // measurement covariance matrx
    MatrixXd R_;

    KalmanFilter();

    virtual ~KalmanFilter();
    /*
     * Reminder:
     *   - what is virtual function in CPP
     *      In OOP, a virtual function is an inheritable and overridable function for
     *      which dynamic dispatch is facilitated. This concept is important part of
     *      runtime polymorphism of OOP.
     *
     *      The concept of the virtual function solves the following problem:
     *       In OOP, when a derived class iherits from a base class, an object of the
     *       derived class may be reffered to via a pointer or reference of the
     *       base class type instead of the derived class type. If there are base class
     *       methods overriden by the derived class, the method actually called by such
     *       reference or pointer can be bound either 'early' (by the compiler),
     *       or 'late' (by the runtime system of the language), according to the actual
     *       type of the object reffered to.
     *
     *       Virtual functions are the one that are resolved 'late'. If the function is
     *       'virtual' in the base class, the most-derived class's implementation of the
     *       function is called according to the actual type of the object referred to,
     *       regardless of the declared type of the pointer or reference. If it is not
     *       'virtual', the method is resolved 'early' and the function called is selected
     *       according to the declared type of the pointer or reference.
     *
     *       Virtual functions allow a program to call methods that don't necessarily even
     *       exist at the moment the code is compiled.
     *
     *       In C++ virtual methos are declared by prepending the virtual keyword to the
     *       function's declaration in the base class. This modifier is inherited by all
     *       implementations of that method in the derived classes, meaning that they can
     *       continue to over-ride each other and be late-bound.
     *
     *
     *   - what is dynamic dispatch then
     *       Dynamic dispathc is calling the function based on runtime type of the object
     *       (by the objects 'real-type' and not by type of the reference).
     *
     *
     *   - what was the subtelty with virtual desctructors again
     *     We need virtual destructors when we want to delete an instance of the derived
     *     class via pointer of type pointer to base class.
     *
     */

    /**
     * Predict the state and the state covariance using the
     * process model
     */
    void Predict();

    /**
     * Update the state based on the newest measurement
     * @param z The measurement at k+1
     */
    void Update(const VectorXd &z);

};


#endif //KALMAN_FILTER_EXAMPLES_KALMAN_FILTER_H
