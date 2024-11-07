#ifndef SHEETFLATTENCORE_H
#define SHEETFLATTENCORE_H

#include <GeomAbs_SurfaceType.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <GeomConvert_SurfToAnaSurf.hxx>
#include <GeomConvert_CurveToAnaCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>

#include "ModelIO/ModelIO.h"
#include "SheetFlattenOnestep.h"

#include <iostream>
#include <vector>
using namespace std;

void BuildDocStd(const TopoDS_Shape& shape)
{
	Handle(TDocStd_Application) app = new TDocStd_Application;
	Handle(TDocStd_Document) doc;
	app->NewDocument("BinXCAF", doc);

	TDF_Label mainLab = doc->Main();
	Handle(XCAFDoc_ShapeTool)ST = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	ST->AddShape(shape);

	ModelIO modelio;
	modelio.Write("F:\\test.stl", doc, GeomSTL);
	//modelio.Write("F:\\test.stp", doc, GeomSTP);

	app->Close(doc);
}


class SheetFlattenCore
{
public:
	SheetFlattenCore()
	{
		aMeshParams.Deflection = 0.02;
		aMeshParams.Angle = 0.1;
		aMeshParams.Relative = Standard_True;
		aMeshParams.InParallel = Standard_True;
		aMeshParams.MinSize = Precision::Confusion();
		aMeshParams.InternalVerticesMode = Standard_True;
		aMeshParams.ControlSurfaceDeflection = Standard_True;
		//aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;

		//CurveDeflection = 0.001;
		//AngularDeflection = 1 / M_PI_4;
	}
public:
	bool Read(const string& filePath);
	int Perform();

private:
	int FlattenSurface(const TopoDS_Face& face);
	int FlattenSurface(const TopoDS_Face& face, TopoDS_Face& flattenFace);
	void FlattenCylinder(const TopoDS_Face& face, TopoDS_Face& flattenFace);
	TopoDS_Wire FlattenFaceEdge(const TopoDS_Face& face);

private:
	Handle(TDocStd_Document) doc;
	TopoDS_Compound aCompound;
	BRep_Builder aBuilder;
	IMeshTools_Parameters aMeshParams;

};



inline bool SheetFlattenCore::Read(const string& filePath)
{
	ModelIO* modelIO = new ModelIO();
	if (modelIO->Read(filePath))
	{
		doc = modelIO->GetDoc();
		delete modelIO;
		return true;
	}
	else
	{
		delete modelIO;
		return false;
	}
}

inline int SheetFlattenCore::Perform()
{
	Handle(XCAFDoc_ShapeTool)ST = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	TDF_LabelSequence Labels;
	ST->GetFreeShapes(Labels);

	aBuilder.MakeCompound(aCompound);

	for (int idx = 1; idx <= Labels.Length(); ++idx)
	{
		TDF_Label theLabel = Labels.ChangeValue(idx);
		TopoDS_Shape objShape;
		ST->GetShape(theLabel, objShape);

		BRepMesh_IncrementalMesh(objShape, aMeshParams);

		for (TopExp_Explorer faceExp(objShape, TopAbs_FACE); faceExp.More(); faceExp.Next())
		{
			const TopoDS_Face& objFace = TopoDS::Face(faceExp.Current());
			TopoDS_Face resFace;

			//FlattenSurface(objFace, resFace);
			FlattenSurface(objFace);
		}
	}

	//BuildDocStd(aCompound);

	return 0;
}
inline int SheetFlattenCore::FlattenSurface(const TopoDS_Face& face, TopoDS_Face& flattenFace)
{
	Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
	GeomConvert_SurfToAnaSurf surf2AnaSurf(surf);
	Handle(Geom_Surface) convertSurf = surf2AnaSurf.ConvertToAnalytical(Precision::Confusion());

	if (convertSurf)
	{
		TopoDS_Face aFace = BRepBuilderAPI_MakeFace(surf, FlattenFaceEdge(face));
		//TopoDS_Face aFace = BRepBuilderAPI_MakeFace(surf, Precision::Confusion());
		//aBuilder.Add(aCompound, aFace);

		//BRepAdaptor_Surface faceAdaptor(aFace, Standard_True);
		//GeomAbs_SurfaceType surfType = faceAdaptor.GetType();	

		//cout << surfType << endl;

		//TopExp_Explorer Ex(aFace, TopAbs_EDGE);

		//for (; Ex.More(); Ex.Next()) {
		//	TopoDS_Edge E = TopoDS::Edge(Ex.Current());
		//	Standard_Real cf, cl;
		//	Handle(Geom2d_Curve) C2d = BRep_Tool::CurveOnSurface(E, aFace, cf, cl);
		//	C2d->DumpJson(cout);
		//	cout << endl;
		//}


		//BuildDocStd(aCompound);
	}

	//BRepAdaptor_Surface faceAdaptor(face, Standard_True);
	//GeomAbs_SurfaceType surfType = faceAdaptor.GetType();	

	return 0;
}

inline TopoDS_Wire SheetFlattenCore::FlattenFaceEdge(const TopoDS_Face& face)
{
	TopExp_Explorer aWireExp(face, TopAbs_WIRE);
	BRepBuilderAPI_MakeWire mkWire;

	for (aWireExp; aWireExp.More(); aWireExp.Next())
	{
		//获取每条wire
		TopoDS_Wire aTempWire = TopoDS::Wire(aWireExp.Current());
		//获取Edge

		TopExp_Explorer aEdgeExp(aTempWire, TopAbs_EDGE);
		for (aEdgeExp; aEdgeExp.More(); aEdgeExp.Next()) 
		{
			TopoDS_Edge aTempEdge = TopoDS::Edge(aEdgeExp.Current());

			// 获取与边相关联的顶点
			TopoDS_Vertex v1, v2;
			TopExp::Vertices(aTempEdge, v1, v2);

			Standard_Real aPFirst, aPLast;
			Handle(Geom_Curve) aCurve = BRep_Tool::Curve(aTempEdge, aPFirst, aPLast);

			GeomConvert_CurveToAnaCurve curve2AnaCurve(aCurve);
			Handle(Geom_Curve) convertCurve;
			Standard_Real aNewFirst, aNewLast;
			double tol = Precision::Confusion();
			curve2AnaCurve.ConvertToAnalytical(1.0, convertCurve, aPFirst, aPLast, aNewFirst, aNewLast);
			TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(convertCurve, v1,v2);

			mkWire.Add(newEdge);
			aBuilder.Add(aCompound, newEdge);
		}


		//if (mkWire.IsDone())
		//{
		//	TopoDS_Wire wire = mkWire.Wire();
		//	TopoDS_Face aFace = BRepBuilderAPI_MakeFace(wire).Face();
		//	aBuilder.Add(aCompound, aFace);
		//}
	}

	if (mkWire.IsDone())
	{
		return mkWire.Wire();
	}

	return mkWire.Wire();

	
}

inline void SheetFlattenCore::FlattenCylinder(const TopoDS_Face& face, TopoDS_Face& flattenFace)
{

}

inline int SheetFlattenCore::FlattenSurface(const TopoDS_Face& face)
{
	SheetFlattenOneStep sfOneStep;
	sfOneStep.Init(face);
	sfOneStep.ExportNas("F:/flat.nas");
	return 0;
}



#endif
