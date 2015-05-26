#ifndef _DEPTH_MAP_FILE_IO_
#define _DEPTH_MAP_FILE_IO_
#include <vector>
#include <fstream>
template <class DataType>
class DepthmapFileIOBase
{
public:
	DepthmapFileIOBase();
	void SetVersion(int v){ Version  = v; }
	void SetNearFar(double n, double f){Near = n;Far = f;}
	void SetWidthHeight(int w, int h) {Width = w; Height = h;}
	void SetData(std::vector<DataType> & data);

	void SaveTofile(char* file_name);
	void LoadFromfile(char* file_name);
protected:
	
	float Version;
	double Near,Far;
	char DataTypeName[8];
	int Width, Height;
	std::vector<DataType> data;
};




//////////////////////////////////////////////////////////////////////////

template <class DataType>
class DepthmapFileIO
{
private:
	DepthmapFileIO(){}
};
template<>
class DepthmapFileIO<float> : public DepthmapFileIOBase<float>
{
public:
	DepthmapFileIO()
	{
		memset(DataTypeName, 0, 8);
		strcpy(DataTypeName, "float");
	};
};
template<>
class DepthmapFileIO<double>: public DepthmapFileIOBase<double>
{
public:
	DepthmapFileIO()
	{
		memset(DataTypeName, 0, 8);
		strcpy(DataTypeName, "double");
	};
};




//////////////////////////////////////////////////////////////////////////
template <class DataType>
DepthmapFileIOBase<DataType>::DepthmapFileIOBase()
{
	Version = 1.0;
}

template <class DataType>
void DepthmapFileIOBase<DataType>::SaveTofile(char* file_name)
{
	std::ofstream out(file_name,  std::ios::binary);
	
	if (!out.is_open())
	{
		std::cout << "File not open, cannot save.\n";
		exit(0);
	}
	out.write((char const*)&Version, 4);
	out.write((char const*)DataTypeName, 8);
	out.write((char const*)&Near, 8);out.write((char const*)&Far, 8);
	out.write((char const*)&Width, 4);out.write((char const*)&Height, 4);

	double range = Far - Near;
	for (int i = 0; i < data.size(); i++)
	{
		DataType d = 0.0;
		d = (data[i] - Near)/range;
		out.write((char const*)&d, sizeof(DataType));
	}

	out.close();
}


template <class DataType>
void DepthmapFileIOBase<DataType>::LoadFromfile(char* file_name)
{
	std::ifstream in(file_name, std::ios::binary);
	if (!in.is_open())
	{
		std::cout << "File not open, cannot laod.\n";
		exit(0);
	}

	in.read((char *)&Version, 4);
	in.read((char *)DataTypeName, 8);
	if (0==strcmp(DataTypeName, "float"))
	{
		if (typeid(double).hash_code() == typeid(DataType).hash_code())
		{
			std::cout << "Warning! File encode with float, but read and store as double in memory, nothing wrong but wasting.\n";
		}
	}
	else if (0==strcmp(DataTypeName, "double"))
	{
		if (typeid(float).hash_code() == typeid(DataType).hash_code())
		{
			std::cout << "Warning! File encode with double, but read as double, May lose precision.\n";
		}
	}
	else
	{
		std::cout << "Unsupported data type, float and double only!\n";
		exit(0);
	}
	in.read((char *)&Near, 8);in.read((char *)&Far, 8);
	in.read((char *)&Width, 4);in.read((char *)&Height, 4);

	double range = Far - Near;
	int size = Width * Height;
	int data_unit_size = (strcmp(DataTypeName, "float")==0) ? 4 : 8;

	for (int i = 0; i < size; i++)
	{
		DataType d = 0.0;
		in.read((char *)&d, data_unit_size);
		data.push_back(d);
	}

	in.close();
}


template <class DataType>
void DepthmapFileIOBase<DataType>::SetData(std::vector<DataType> & _data)
{
	if (_data.size() != Width*Height)
	{
		std::cout << "Data size not match!\n";
		exit(0);
	}
	data = _data;
}


#endif