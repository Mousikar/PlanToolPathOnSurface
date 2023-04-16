#include <pcl/io/vtk_lib_io.h>
#include <vtkPLYReader.h>
#include <vtkSTLWriter.h>
 
int main(int argc, char** argv)
{
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName("./src/data/result.ply");
	reader->Update();
 
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData = reader->GetOutput();
	polyData->GetNumberOfPoints();
 
	vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
	writer->SetInputData(polyData);
	writer->SetFileName("./src/data/curvenoisefilter.stl");
	writer->Write();
 
	return 0;
}