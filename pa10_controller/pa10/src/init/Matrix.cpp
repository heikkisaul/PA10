#include <vector>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <stdexcept>
#include <iostream>

#include "Matrix.hpp"

Matrix::Matrix() 
	: num_row{4}, num_col(4) {
	matrix = init_matrix(num_row, num_col);
}

Matrix::Matrix(int num_row, int num_col)
	: num_row{num_row}, num_col{num_col} {
	matrix = init_matrix(num_row, num_col);
}

Matrix::~Matrix()
{
	delete_matrix(matrix, num_row);
}

Matrix& Matrix::operator=(const Matrix& rhs)
{
	delete_matrix(matrix, num_row);
	num_row = rhs.num_row;
	num_col = rhs.num_col;
	matrix = init_matrix(num_row, num_col);

	for(int i = 0; i < num_row; ++i) {
		for(int j = 0; j < num_col; ++j) {
			matrix[i][j] = rhs.matrix[i][j];
		}
	}

	return *this;
}

Matrix Matrix::operator+(const Matrix& rhs)
{
	if(rhs.num_row != num_row || rhs.num_col != num_col) throw std::invalid_argument("Incompatable matricies for this operation.");

	Matrix return_matrix(num_row, num_col);

	for(int i = 0; i < num_row; ++i) {
		for(int j = 0; j < num_col; ++j) {
			return_matrix.matrix[i][j] = matrix[i][j] + rhs.matrix[i][j];
		}
	}
	return return_matrix;
}

Matrix Matrix::operator*(const Matrix& rhs)
{
	if(num_col != rhs.num_row) throw std::invalid_argument("Incompatable matricies for this operation.");

	Matrix return_matrix(num_row, rhs.num_col);
	double sum = 0;

	for(int i = 0; i < num_row; ++i) {
		for(int j = 0; j < rhs.num_col; ++j) {
			sum = 0;
			for(int k = 0; k < num_col; ++k) {
				sum += matrix[i][k] * rhs.matrix[k][j];
			}
			return_matrix.matrix[i][j] = sum;
		}
	}

	return return_matrix;
}

Matrix& Matrix::operator*=(const Matrix& rhs)
{
	//this is p shit tbh fampai
	//mebbe I'll come fix this later
	//gonna drop a TODO here 
	return *this = *this * rhs;
}

void Matrix::set_value(int row, int col, double value)
{
	if(row >= num_row) throw std::invalid_argument("Row index out of bound.");
	if(col >= num_col) throw std::invalid_argument("Column index out of bound.");

	matrix[row][col] = value;
}

double Matrix::get_value(int row, int col) const
{
	if(row >= num_row) throw std::invalid_argument("Row index out of bound.");
	if(col >= num_col) throw std::invalid_argument("Column index out of bound.");

	return matrix[row][col];
}

int Matrix::get_num_row() const
{
    return num_row;
}

int Matrix::get_num_col() const
{
    return num_col;
}

void Matrix::print()
{
	for(int i = 0; i < num_row; ++i) {
		for(int j = 0; j < num_col; ++j) {
			printf("%8.2lf", matrix[i][j]);
		}
		std::cout << std::endl;
	}
}

double **Matrix::init_matrix(int num_row, int num_col)
{
	double **return_matrix = new double *[num_row];

	for(int i = 0; i < num_row; ++i) {
		return_matrix[i] = new double[num_col];
	}
	
	return return_matrix;
}

void Matrix::delete_matrix(double **matrix, int num_row)
{
	for(int i = 0; i < num_row; ++i) {
		delete[] matrix[i];
	}
	delete[] matrix;
}
