#ifndef MATRIX_HPP
#define MATRIX_HPP

class Matrix
{
	public:
		Matrix();
		Matrix(int num_row, int num_col);
		~Matrix();

		Matrix& operator=(const Matrix& rhs);
		Matrix operator+(const Matrix& rhs);
		Matrix operator*(const Matrix& rhs);
		Matrix& operator*=(const Matrix& rhs);

		void set_value(int row, int col, double value);
		double get_value(int row, int col) const;
		int get_num_row() const;
		int get_num_col() const;
		void print();

	private:
		double **matrix;
		int num_row;
		int num_col;
		double **init_matrix(int num_row, int num_col);
		void delete_matrix(double **matrix, int num_row);
};


#endif // MATTRIX_HPP
