/*
 * mat.h
 * 
 * Created : 12/31/2018
 *  Author : n-is
 *   email : 073bex422.nischal@pcampus.edu.np
 */

#include "mat.h"


Mat::Mat(uint8_t rows, uint8_t columns)
{
        if (rows > MAX_MATRIX_ROWS || columns > MAX_MATRIX_COLS) {
                _Error_Handler(__FILE__, __LINE__);
        }
        rows_ = rows;
        cols_ = columns;
        
        for (uint8_t i = 0; i < MAX_MATRIX_ROWS; ++i) {
                for (uint8_t j = 0; j < MAX_MATRIX_COLS; ++j) {
                        matrix_[i][j] = 0;
                }
        }
}

Mat::Mat(const Mat &m) {
        rows_ = m.rows_;
        cols_ = m.cols_;
        
        for (uint8_t i = 0; i < MAX_MATRIX_ROWS; ++i) {
                for (uint8_t j = 0; j < MAX_MATRIX_COLS; ++j) {
                        matrix_[i][j] = m.matrix_[i][j];
                }
        }
}

void swap(Mat &first, Mat &second)
{
        swap_Element(first.rows_, second.rows_);
        swap_Element(first.cols_, second.cols_);
        for (uint8_t i = 0; i < MAX_MATRIX_ROWS; ++i) {
                for (uint8_t j = 0; j < MAX_MATRIX_COLS; ++j) {
                        swap_Element(first.matrix_[i][j], second.matrix_[i][j]);
                }
        }
}

Mat& Mat::operator+=(const Mat &rhs)
{
        if (rhs.rows_ != rows_ || rhs.cols_ != cols_) {
                _Error_Handler(__FILE__, __LINE__);
        }

        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < cols_; ++j) {
                        matrix_[i][j] += rhs.matrix_[i][j];
                }
        }

        return *this;
}

Mat& Mat::operator-=(const Mat &rhs)
{
        if (rhs.rows_ != rows_ || rhs.cols_ != cols_) {
                _Error_Handler(__FILE__, __LINE__);
        }

        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < cols_; ++j) {
                        matrix_[i][j] -= rhs.matrix_[i][j];
                }
        }

        return *this;
}

Mat& Mat::operator*=(const Mat &rhs)
{
        *this = mult(rhs);
        return *this;
}

Mat& Mat::operator*=(const Vec3<float> &rhs)
{
        Mat vec(3,1);
        vec.at(0,0) = rhs.getX();
        vec.at(1,0) = rhs.getY();
        vec.at(2,0) = rhs.getZ();

        *this = mult(vec);
        return *this;
}

Mat Mat::mult(const Mat &m)
{
        if (cols_ != m.rows_) {
                _Error_Handler(__FILE__, __LINE__);
        }

        Mat product(rows_, m.cols_);
        float sum;
        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < m.cols_; ++j) {
                        sum = 0;
                        for (uint8_t k = 0; k < cols_; ++k) {
                                sum += matrix_[i][k] * m.matrix_[k][j];
                        }
                        product.matrix_[i][j] = sum;
                }
        }

        return product;
}

Mat Mat::mult_EW(float num)
{
        Mat temp(rows_, cols_);
        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < cols_; ++j) {
                        temp.matrix_[i][j] *= num;
                }
        }

        return temp;
}

Mat Mat::transpose()
{
        Mat trans(cols_, rows_);
        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < cols_; ++j) {
                        trans.matrix_[i][j] = matrix_[j][i];
                }
        }
        return trans;
}

Mat Mat::eye(uint8_t n)
{
        Mat I(n,n);
        I.fill(0);
        for (uint8_t i = 0; i < n; ++i) {
                I.matrix_[i][i] = 1;
        }
        return I;
}

bool Mat::is_Zero() const
{
        for (uint8_t i = 0; i < rows_; ++i) {
                for (uint8_t j = 0; j < cols_; ++j) {
                        if (fabs(matrix_[i][j]) > 1e-8f) {
                                return false;
                        }
                }
        }
        return true;
}

void Mat::swap_Rows(uint8_t a, uint8_t b)
{
        if (a > rows_ || b > rows_) {
                _Error_Handler(__FILE__, __LINE__);
        }
        if (a == b) {
                return;
        }

        for (uint8_t j = 0; j < cols_; j++) {
                float tmp = matrix_[a][j];
                matrix_[a][j] = matrix_[b][j];
                matrix_[b][j] = tmp;
        }
}

void Mat::swap_Cols(size_t a, size_t b)
{
        if (a > cols_ || b > cols_) {
                _Error_Handler(__FILE__, __LINE__);
        }
        if (a == b) {
                return;
        }

        for (size_t i = 0; i < rows_; i++) {
                float tmp = matrix_[i][a];
                matrix_[i][a] = matrix_[i][b];
                matrix_[i][b] = tmp;
        }
}

/**
 * inverse based on LU factorization with partial pivotting
 */
bool Mat::inv(Mat &inv) const
{
        if (rows_ != cols_ || rows_ != inv.rows() || inv.rows() != inv.cols()) {
                _Error_Handler(__FILE__, __LINE__);
        }
        Mat L = Mat::eye(rows_);
        Mat U = *this;
        Mat P = Mat::eye(rows_);

        //printf("A:\n"); print();

        // for all diagonal elements
        for (size_t n = 0; n < rows_; n++) {

                // if diagonal is zero, swap with row below
                if (fabs(U.at(n, n)) < 1e-8f) {
                        //printf("trying pivot for row %d\n",n);
                        for (size_t i = n + 1; i < rows_; i++) {
                                //printf("\ttrying row %d\n",i);
                                if (fabs(U.at(i, n)) > 1e-8f) {
                                        //printf("swapped %d\n",i);
                                        U.swap_Rows(i, n);
                                        P.swap_Rows(i, n);
                                        L.swap_Rows(i, n);
                                        L.swap_Cols(i, n);
                                        break;
                                }
                        }
                }

                // failsafe, return zero matrix
                if (fabs(U.at(n, n)) < 1e-8f) {
                        return false;
                }

                // for all rows below diagonal
                for (size_t i = (n + 1); i < rows_; i++) {
                        L.at(i, n) = U.at(i, n) / U.at(n, n);

                        // add i-th row and n-th row
                        // multiplied by: -a(i,n)/a(n,n)
                        for (size_t k = n; k < rows_; k++) {
                                U.at(i, k) -= L.at(i, n) * U.at(n, k);
                        }
                }
        }

        //printf("L:\n"); L.print();
        //printf("U:\n"); U.print();

        // solve LY=P*I for Y by forward subst
        //SquareMatrix<Type, M> Y = P;

        // for all columns of Y
        for (size_t c = 0; c < rows_; c++) {
                // for all rows of L
                for (size_t i = 0; i < rows_; i++) {
                        // for all columns of L
                        for (size_t j = 0; j < i; j++) {
                                // for all existing y
                                // subtract the component they
                                // contribute to the solution
                                P.at(i, c) -= L.at(i, j) * P.at(j, c);
                        }

                        // divide by the factor
                        // on current
                        // term to be solved
                        // Y(i,c) /= L(i,i);
                        // but L(i,i) = 1.0
                }
        }

        //printf("Y:\n"); Y.print();

        // solve Ux=y for x by back subst
        //SquareMatrix<Type, M> X = Y;

        // for all columns of X
        for (size_t c = 0; c < rows_; c++) {
                // for all rows of U
                for (size_t k = 0; k < rows_; k++) {
                        // have to go in reverse order
                        size_t i = rows_ - 1 - k;

                        // for all columns of U
                        for (size_t j = i + 1; j < rows_; j++) {
                                // for all existing x
                                // subtract the component they
                                // contribute to the solution
                                P.at(i, c) -= U.at(i, j) * P.at(j, c);
                        }

                        // divide by the factor
                        // on current
                        // term to be solved
                        //
                        // we know that U(i, i) != 0 from above
                        P.at(i, c) /= U.at(i, i);
                }
        }

        //check sanity of results
        for (size_t i = 0; i < rows_; i++) {
                for (size_t j = 0; j < rows_; j++) {
                        if (!isfinite(P.at(i, j))) {
                                return false;
                        }
                }
        }
        //printf("X:\n"); X.print();
        inv = P;
        return true;
}
