#ifndef MODELIO_H
#define MODELIO_H

#include <BinXCAFDrivers.hxx>
#include <XCAFApp_Application.hxx>
#include <TDocStd_Document.hxx>
#include <BRepTools.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <BRep_Builder.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepAlgoAPI_Cut.hxx>

#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDF_Attribute.hxx>
#include <TDF_LabelMap.hxx>
#include <TDF_LabelSequence.hxx>
#include <IFSelect_ReturnStatus.hxx>

#include <XCAFDoc_Color.hxx>
#include <BRepMesh_IncrementalMesh.hxx>

#include <STEPControl_StepModelType.hxx>
#include <UnitsMethods.hxx>
#include <Prs3d_Drawer.hxx>

#include <IGESCAFControl_Reader.hxx>
#include <IGESCAFControl_Writer.hxx>

#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>

#include <RWGltf_CafReader.hxx>
#include <RWGltf_CafWriter.hxx>

#include <StlAPI_Writer.hxx>
#include <StlAPI_Reader.hxx>

#include <string>

using namespace std;

#define UPPERBOUND 999999

enum GeomType
{
	GeomSTP,
	GeomIGS,
	GeomSTL,
	GeomBREP,
	GeomGLTF
};

class ModelIO
{
public:
	ModelIO() {}
	bool Read(const string &fileName, const GeomType &modelType = GeomSTP);
	bool Write(const string &fileName, Handle(TDocStd_Document) doc, const GeomType &modelType = GeomSTP);
	Handle(TDocStd_Document) GetDoc();

private:
	bool ReadIGS(const string &fileName);
	bool ReadSTP(const string &fileName);
	bool ReadBREP(const string &filename);
	bool ReadSTL(const string &filename);
	bool ReadGLTF(const string &filename);

	bool WriteSTP(const string &fileName, Handle(TDocStd_Document) doc);
	bool WriteIGS(const string &fileName, Handle(TDocStd_Document) doc);
	bool WriteBREP(const string &fileName, Handle(TDocStd_Document) doc);
	bool WriteSTL(const string &fileName, Handle(TDocStd_Document) doc);
	bool WriteGLTF(const string &fileName, Handle(TDocStd_Document) doc);

	GeomType modelFormat;
	Handle(TDocStd_Document) mainDoc;
};

inline Handle(TDocStd_Document) ModelIO::GetDoc()
{
	return mainDoc;
}

inline bool ModelIO::Read(const string &fileName, const GeomType &modelType)
{
	modelFormat = modelType;

	switch (modelType)
	{
	case GeomSTP:
		return ReadSTP(fileName);
	case GeomIGS:
		return ReadIGS(fileName);
	case GeomSTL:
		return ReadSTL(fileName);
	case GeomBREP:
		return ReadBREP(fileName);
	case GeomGLTF:
		return ReadGLTF(fileName);
	default:
		break;
	}

	return false;
}

inline bool ModelIO::Write(const string &fileName, Handle(TDocStd_Document) doc, const GeomType &modelType)
{
	switch (modelType)
	{
	case GeomSTP:
		return WriteSTP(fileName, doc);
	case GeomIGS:
		return WriteIGS(fileName, doc);
	case GeomSTL:
		return WriteSTL(fileName, doc);
	case GeomBREP:
		return WriteBREP(fileName, doc);
	case GeomGLTF:
		return WriteGLTF(fileName, doc);
	default:
		break;
	}

	return false;
}

inline bool ModelIO::ReadIGS(const string &fileName)
{
	IGESCAFControl_Reader aIgesReader;
	aIgesReader.SetColorMode(true);
	aIgesReader.SetNameMode(true);

	IFSelect_ReturnStatus status = aIgesReader.ReadFile(fileName.c_str());

	Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
	BinXCAFDrivers::DefineFormat(anApp);
	anApp->NewDocument("BinXCAF", mainDoc);

	if (status == IFSelect_RetDone)
	{
		aIgesReader.Transfer(mainDoc);
		return true;
	}

	return false;
}

inline bool ModelIO::ReadSTP(const string &fileName)
{

	Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
	BinXCAFDrivers::DefineFormat(anApp);
	anApp->NewDocument("BinXCAF", mainDoc);

	STEPCAFControl_Reader aStepReader;
	aStepReader.SetColorMode(true);
	aStepReader.SetNameMode(true);

	IFSelect_ReturnStatus status = aStepReader.ReadFile(fileName.c_str());

	if (status == IFSelect_RetDone)
	{
		aStepReader.Transfer(mainDoc);
		return true;
	}

	return false;
}

inline bool ModelIO::ReadBREP(const string &filename)
{
	TopoDS_Shape aShape;
	BRep_Builder aBuilder;

	if (!BRepTools::Read(aShape, filename.c_str(), aBuilder))
	{
		return false;
	}	

	mainDoc = new TDocStd_Document("BRep");

	Handle(XCAFDoc_ShapeTool) ST = XCAFDoc_DocumentTool::ShapeTool(mainDoc->Main());

	ST->AddShape(aShape);

	return true;
}

inline bool ModelIO::ReadSTL(const string &filename)
{
	TopoDS_Shape aShape;

	StlAPI_Reader anStlReader;

	if (!anStlReader.Read(aShape, filename.c_str()))
	{
		return false;
	}

	mainDoc = new TDocStd_Document("STL");

	Handle(XCAFDoc_ShapeTool) ST = XCAFDoc_DocumentTool::ShapeTool(mainDoc->Main());
	Handle(XCAFDoc_ColorTool) CT = XCAFDoc_DocumentTool::ColorTool(mainDoc->Main());
	Quantity_ColorRGBA aColor(0.644479692f, 0.644479692f, 1.00000000f, 1.00000000f);

	TDF_Label aLabel = ST->AddShape(aShape);
	CT->SetColor(aShape, aColor, XCAFDoc_ColorSurf);

	TDataStd_Name::Set(aLabel, "STL_Shape");

	return true;
}

inline bool ModelIO::ReadGLTF(const string &filename)
{
	Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
	BinXCAFDrivers::DefineFormat(anApp);
	anApp->NewDocument("BinXCAF", mainDoc);

	RWGltf_CafReader aReader;
	Standard_Real aSystemUnitFactor = UnitsMethods::GetCasCadeLengthUnit() * 0.001;
	aReader.SetSystemLengthUnit(aSystemUnitFactor);
	aReader.SetSystemCoordinateSystem(RWMesh_CoordinateSystem_Zup);
	aReader.SetDocument(mainDoc);
	aReader.SetParallel(Standard_True);
	Message_ProgressRange theProgress;

	if (aReader.Perform(filename.c_str(), theProgress))
	{
		Handle(XCAFDoc_ColorTool) CT = XCAFDoc_DocumentTool::ColorTool(mainDoc->Main());
		Quantity_ColorRGBA aColor(0.644479692f, 0.644479692f, 1.00000000f, 1.00000000f);
		CT->SetColor(mainDoc->Main(), aColor, XCAFDoc_ColorSurf);
		return true;
	}
	else
	{
		return false;
	}
}
// ------------------------------------------
// Write Files
// ------------------------------------------
inline bool ModelIO::WriteSTP(const string &fileName, Handle(TDocStd_Document) doc)
{
	if (modelFormat == GeomGLTF)
	{
		return false;
	}

	STEPControl_StepModelType mode = STEPControl_AsIs;

	STEPCAFControl_Writer aWriter;
	aWriter.SetColorMode(true);
	aWriter.SetNameMode(true);

	// Translating document (conversion) to STEP
	if (!aWriter.Transfer(doc, mode)) {
		return false;
	}
	// Writing the File
	IFSelect_ReturnStatus status = aWriter.Write(fileName.c_str());

	// 检查是否成功写入
	if (status != IFSelect_RetDone) {
		return false;		
	}

	return true;
}
inline bool ModelIO::WriteIGS(const string &fileName, Handle(TDocStd_Document) doc)
{
	if (modelFormat == GeomGLTF)
	{
		return false;
	}

	IGESCAFControl_Writer aWriter;
	aWriter.SetColorMode(true);
	aWriter.SetNameMode(true);

	return aWriter.Perform(doc, fileName.c_str());
}

inline bool ModelIO::WriteBREP(const string &fileName, Handle(TDocStd_Document) doc)
{
	Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	TDF_LabelSequence aRootLabels;
	aShapeTool->GetFreeShapes(aRootLabels);

	TopoDS_Compound aCompound;
	BRep_Builder    aBuildTool;
	aBuildTool.MakeCompound(aCompound);
	for (TDF_LabelSequence::Iterator aRootIter(aRootLabels); aRootIter.More(); aRootIter.Next())
	{
		const TDF_Label& aRootLabel = aRootIter.Value();
		TopoDS_Shape aRootShape;
		if (XCAFDoc_ShapeTool::GetShape(aRootLabel, aRootShape))
		{
			aBuildTool.Add(aCompound, aRootShape);
		}
	}

	return BRepTools::Write(aCompound, fileName.c_str());
}


inline bool ModelIO::WriteSTL(const string &fileName, Handle(TDocStd_Document) doc)
{
	cout << "WriteSTL " << endl;
	Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	TDF_LabelSequence aRootLabels;
	aShapeTool->GetFreeShapes(aRootLabels);

	TopoDS_Compound aCompound;
	BRep_Builder    aBuildTool;
	aBuildTool.MakeCompound(aCompound);
	for (TDF_LabelSequence::Iterator aRootIter(aRootLabels); aRootIter.More(); aRootIter.Next())
	{
		const TDF_Label& aRootLabel = aRootIter.Value();
		TopoDS_Shape aRootShape;
		if (XCAFDoc_ShapeTool::GetShape(aRootLabel, aRootShape))
		{
			aBuildTool.Add(aCompound, aRootShape);
		}
	}

	// perform meshing
	Handle(Prs3d_Drawer) aDrawer = new Prs3d_Drawer(); // holds visualization defaults
	BRepMesh_IncrementalMesh anAlgo;
	anAlgo.ChangeParameters().Deflection = 0.2;
	anAlgo.ChangeParameters().Angle = 20.0 * M_PI / 180.0; // 20 degrees
	anAlgo.ChangeParameters().InParallel = true;
	anAlgo.SetShape(aCompound);
	anAlgo.Perform();

	

	StlAPI_Writer anStlWriter;
	//anStlWriter.ASCIIMode() = false;

	cout << "Start Writing STL." << endl;

	if (anStlWriter.Write(aCompound, fileName.c_str()))
	{
		return true;
	}
	return false;
}

inline bool ModelIO::WriteGLTF(const string &fileName, Handle(TDocStd_Document) doc)
{
	Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	TDF_LabelSequence aRootLabels;
	aShapeTool->GetFreeShapes(aRootLabels);

	TopoDS_Compound aCompound;
	BRep_Builder    aBuildTool;
	aBuildTool.MakeCompound(aCompound);
	for (TDF_LabelSequence::Iterator aRootIter(aRootLabels); aRootIter.More(); aRootIter.Next())
	{
		const TDF_Label& aRootLabel = aRootIter.Value();
		TopoDS_Shape aRootShape;
		if (XCAFDoc_ShapeTool::GetShape(aRootLabel, aRootShape))
		{
			aBuildTool.Add(aCompound, aRootShape);
		}
	}

	// perform meshing
	Handle(Prs3d_Drawer) aDrawer = new Prs3d_Drawer(); // holds visualization defaults
	BRepMesh_IncrementalMesh anAlgo;
	anAlgo.ChangeParameters().Deflection = 0.2;
	anAlgo.ChangeParameters().Angle = 20.0 * M_PI / 180.0; // 20 degrees
	anAlgo.ChangeParameters().InParallel = true;
	anAlgo.SetShape(aCompound);
	anAlgo.Perform();

	TColStd_IndexedDataMapOfStringString aMetadata;
	Message_ProgressRange theProgress;
	RWGltf_CafWriter aGltfWriter(fileName.c_str(), true);
	// STEP reader translates into mm units by default
	aGltfWriter.ChangeCoordinateSystemConverter().SetInputLengthUnit(0.001);
	aGltfWriter.ChangeCoordinateSystemConverter().SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
	if (aGltfWriter.Perform(doc, aMetadata, theProgress))
	{
		return true;
	}
	return false;
}
#endif
