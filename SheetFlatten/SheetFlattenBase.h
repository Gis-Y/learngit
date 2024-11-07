#pragma once

#define BUILD_OUTSIDE_NX_ENV
#include "OneStep/BodyDes_OnestepUnformSolver.hxx"
#include "SheetFlattenFitting.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TDF_Label.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRepTools.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <TopoDS.hxx>
#include <Poly_PolygonOnTriangulation.hxx>

#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <BinXCAFDrivers.hxx>

#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <CPnts_AbscissaPoint.hxx>

#include <Poly_Triangulation.hxx>
#include <Poly_ArrayOfNodes.hxx>

#include <BRep_TEdge.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_PolygonOnTriangulation.hxx>

#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <Geom2dAPI_Interpolate.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BSplineCurve.hxx>
#ifdef new
#undef new
#endif // new

#include <iostream>
#include <vector>
#include <set>
#include <unordered_map>

#include <map>
#include <math.h>
#include <chrono>

#include <algorithm>    // set_intersection
#include <iterator>    // insert_iterator
#include "SaveDxfFile.h"
#include "DxfCoreData.hxx"

using namespace dxf::core;


using namespace std;

#pragma warning(disable:4267)

#define TOL 1e-3
#define MAP unordered_map

class Normal
{
public:
	double x, y, z;
	Normal()
	{
		x = 0.0;
		y = 0.0;
		z = 1.0;
	}
	Normal(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;

		double norm = sqrt(x*x + y * y + z * z);

		if (norm < 1e-9)
		{
			x = 0.0;
			y = 0.0;
			z = 1.0;
		}
		else
		{
			x = x / norm;
			y = y / norm;
			z = z / norm;
		}
	}
};

class Node
{
public:
	int nid;
	double x;
	double y;
	double z;

	TopoDS_Face face;
	TopoDS_Shape shape;

	Normal nor;
	set<int> adjElems;

	Node()
	{
		nid = 0;
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}

	Node(const double &_x, const double &_y, const double &_z)
	{
		nid = 0;
		x = _x;
		y = _y;
		z = _z;
	}

	Node(const int & _nid, const double &_x, const double &_y, const double &_z)
	{
		nid = _nid;
		x = _x;
		y = _y;
		z = _z;
	}

	Normal GetNormal()
	{
		return nor;
	}

	void GetNormal(double &nx, double &ny, double &nz)
	{
		nx = nor.x;
		ny = nor.y;
		nz = nor.z;
	}

	void SetNormal(double nx, double ny, double nz)
	{
		nor.x = nx;
		nor.y = ny;
		nor.z = nz;
	}

	void addAdjElement(int eid)
	{
		adjElems.insert(eid);
	}

	double DistanceTo(Node _n)
	{
		return fabs(sqrt((x - _n.x) * (x - _n.x) +
			(y - _n.y) * (y - _n.y) +
			(z - _n.z) * (z - _n.z)));
	}
	double SquareDistanceTo(Node _n) const
	{
		return (x - _n.x) * (x - _n.x) +
			(y - _n.y) * (y - _n.y) +
			(z - _n.z) * (z - _n.z);
	}
	void SetCoordinate(gp_Pnt pt)
	{
		x = pt.X();
		y = pt.Y();
		z = pt.Z();
	}

	bool IsSameNode(Node _n)
	{
		if (fabs(x - _n.x) > TOL)
		{
			return false;
		}

		if (fabs(y - _n.y) > TOL)
		{
			return false;
		}

		if (fabs(z - _n.z) > TOL)
		{
			return false;
		}

		return true;
	}

	bool IsSameNode(gp_Pnt _n)
	{
		if (fabs(x - _n.X()) > TOL)
		{
			return false;
		}

		if (fabs(y - _n.Y()) > TOL)
		{
			return false;
		}

		if (fabs(z - _n.Z()) > TOL)
		{
			return false;
		}

		return true;
	}

};

class Element
{
public:
	int eid;
	int pid;
	int n1, n2, n3;
	set<int> adjoinElements;
	TopoDS_Face face;
	TopoDS_Shape shape;

	Element()
	{
		eid = 0;
		pid = 1;
		n1 = 0;
		n2 = 0;
		n3 = 0;
	}
	void ReplaceN1(const int &rep)
	{
		n1 = rep;
	}

	void ReplaceN2(const int &rep)
	{
		n2 = rep;
	}

	void ReplaceN3(const int &rep)
	{
		n3 = rep;
	}
};

class Edge
{
public:
	int v1;
	int v2;
	set<int> fatherElements;

	Edge(const int& _v1, const int& _v2)
	{
		v1 = _v1;
		v2 = _v2;
	}

	void AddElements(int e)
	{
		fatherElements.insert(e);
	}

	bool GetAnotherVertex(const int &vid, int &v_res) const
	{
		if (v1 == vid)
		{
			v_res = v2;
			return true;
		}
		else if (v2 == vid)
		{
			v_res = v1;
			return true;
		}
		return false;
	}

	void ReplaceVertex1(const int &rep)
	{
		v1 = rep;
	}

	void ReplaceVertex2(const int &rep)
	{
		v2 = rep;
	}

	void GetSortVertex(int &_v1, int &_v2)	const
	{
		if (v1 > v2)
		{
			_v1 = v2;
			_v2 = v1;
		}
		else
		{
			_v1 = v1;
			_v2 = v2;
		}
	}

	bool operator<(const Edge & e) const
	{
		int v1ori, v2ori, v1e, v2e;
		GetSortVertex(v1ori, v2ori);
		e.GetSortVertex(v1e, v2e);

		if (v1ori < v1e)
		{
			return true;
		}
		else if (v1ori == v1e && v2ori < v2e)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool operator==(const Edge & e) const
	{
		int v1ori, v2ori, v1e, v2e;
		GetSortVertex(v1ori, v2ori);
		e.GetSortVertex(v1e, v2e);

		if (v1ori == v1e && v2ori == v2e)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

enum FaceOrientation
{
	Clockwise,		// 向量积为正
	Anticlockwise
};


class FlattenFace
{
private:

	MAP<int, Node> nodes;			// id从1开始
	MAP<int, Node> flatNodes;		// Onestep展平结果
	MAP<int, gp_Pnt2d> planeNodes;		// 平移到Z平面结果
	MAP<int, gp_Pnt2d> jigsawNodes;	// 组合到一起的结果
	MAP<int, Element> elements;		// id从1开始
	

	set<int> edgeNodes;	//存储更新完ID的边界节点
	gp_Pnt cogOnestep;
	gp_Pnt2d cogPlane;
	std::vector<std::tuple<point_t, point_t, std::string>> ai;
	std::vector<std::tuple<point_t, double, std::string>> ai1;
	std::vector< std::tuple< point_t, double, double, double, std::string > > ai2;

	int jigsawStartNodeId, jigsawEndNodeId;
public:

	MAP<TopoDS_Edge, TColStd_Array1OfInteger>edges;
	MAP<size_t, TColStd_Array1OfInteger>edgesHash;

	TopoDS_Face face;

	FlattenFace() {}

	FlattenFace(const TopoDS_Face& objFace)
	{
		Init(objFace);
	}

	void Init(const TopoDS_Face& objFace)
	{
		face = objFace;

		TopLoc_Location location;

		Handle(Poly_Triangulation) triFace = BRep_Tool::Triangulation(objFace, location);

		if (triFace.IsNull())
		{
			cout << "triFace is Null" << endl;
			return;
		}

		BuildNodes(triFace->InternalNodes(), location);
		BuildElement(triFace->InternalTriangles());
		this->DumpNasEx("E:/face1.nas");
		BuildEdge(objFace, triFace);

	}

	void BuildNodes(const Poly_ArrayOfNodes& aNodes, const TopLoc_Location& location)
	{
		for (int i = 1; i <= aNodes.Length(); i++)
		{
			gp_Pnt vertex = aNodes.Value(i - 1).Transformed(location.Transformation());
			Node _n;
			_n.nid = i;
			_n.x = vertex.X();
			_n.y = vertex.Y();
			_n.z = vertex.Z();

			nodes.insert(make_pair(i, _n));
		}
	}

	void BuildElement(const Poly_Array1OfTriangle& aTri)
	{
		for (Standard_Integer i = 1; i <= aTri.Length(); i++)
		{
			Standard_Integer nVertexIndex1 = 0;
			Standard_Integer nVertexIndex2 = 0;
			Standard_Integer nVertexIndex3 = 0;

			Poly_Triangle aTriangle = aTri.Value(i);
			aTriangle.Get(nVertexIndex1, nVertexIndex2, nVertexIndex3);

			Element _triElem;
			_triElem.eid = i;
			_triElem.pid = 1;
			_triElem.n1 = nVertexIndex1;
			_triElem.n2 = nVertexIndex2;
			_triElem.n3 = nVertexIndex3;

			elements.insert(make_pair(i, _triElem));
		}
	}

	Standard_Integer findNode(const Poly_ArrayOfNodes& arrayOfNodes, const gp_Pnt& targetPnt) {
		Standard_Integer size = arrayOfNodes.Length();
		for (Standard_Integer i = 1; i <= size; i++) {
			const gp_Pnt& nodePnt = arrayOfNodes.Value(i - 1);
			if (nodePnt.IsEqual(targetPnt, 1e-6)) {
				return i;
			}
		}
		return 0;
	}

	void BuildEdge(const TopoDS_Face& objFace, const Handle(Poly_Triangulation) &triFace)
	{
		//TopLoc_Location location;
		//opencascade::handle<Poly_Triangulation> triFace = BRep_Tool::Triangulation(objFace, location);

		const Poly_ArrayOfNodes& aNodes = triFace->InternalNodes();

		for (TopExp_Explorer edgeExp(objFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());
			TopLoc_Location location;

			Handle(Poly_PolygonOnTriangulation) triEdge = BRep_Tool::PolygonOnTriangulation(objEdge, triFace, location);

			TColStd_Array1OfInteger nodes;

			// 获取失败，重新定义
			if (triEdge.IsNull())
			{
				nodes.Resize(1, 2, FALSE);
				// 获取边的顶点
				TopoDS_Vertex firstVertex, lastVertex;
				TopExp::Vertices(objEdge, firstVertex, lastVertex);

				// 获取顶点的坐标
				gp_Pnt firstVertexPoint = BRep_Tool::Pnt(firstVertex);
				gp_Pnt lastVertexPoint = BRep_Tool::Pnt(lastVertex);


				nodes.SetValue(1, findNode(aNodes, firstVertexPoint));
				nodes.SetValue(2, findNode(aNodes, lastVertexPoint));
			}
			else
			{
				nodes.Resize(1, triEdge->Nodes().Size(), FALSE);
				nodes = triEdge->Nodes();
			}

			edges.insert(make_pair(objEdge, nodes));
			edgesHash.insert(make_pair(hash<TopoDS_Edge>{}(objEdge), nodes));

			for (int i = 1; i <= nodes.Size(); i++)
			{
				edgeNodes.insert(nodes[i]);
			}
		}
	}

	void GetOnestepCOG()
	{
		for (auto edgeId : edgeNodes)
		{
			cogOnestep.SetX(cogOnestep.X() + flatNodes[edgeId].x);
			cogOnestep.SetY(cogOnestep.Y() + flatNodes[edgeId].y);
			cogOnestep.SetZ(cogOnestep.Z() + flatNodes[edgeId].z);
		}

		cogOnestep.SetX(cogOnestep.X() / edgeNodes.size());
		cogOnestep.SetY(cogOnestep.Y() / edgeNodes.size());
		cogOnestep.SetZ(cogOnestep.Z() / edgeNodes.size());
	}

	void GetPlaneCOG()
	{
		for (auto edgeId : edgeNodes)
		{
			cogPlane.SetX(cogPlane.X() + planeNodes[edgeId].X());
			cogPlane.SetY(cogPlane.Y() + planeNodes[edgeId].Y());
		}

		cogPlane.SetX(cogPlane.X() / edgeNodes.size());
		cogPlane.SetY(cogPlane.Y() / edgeNodes.size());
	}


	Node GetNode(const int& nid)
	{
		return nodes[nid];
	}

	Node GetFlatNode(const int& nid)
	{
		return flatNodes[nid];
	}

	gp_Pnt2d GetJigsawNode(const int& nid)
	{
		return jigsawNodes[nid];
	}


	int OneStepFlattenFace()
	{
		KMAS::OnestepUnformConstructor(KMAS::WHOLE_UNFOLD);

		double* coords = new double[nodes.size() * 3];
		int* indices = new int[elements.size() * 4];

		map<int, int> nidMap;
		map<int, int> nidMapReverse;
		int nid = 0;

		for (auto p : nodes)
		{
			Node a = p.second;
		}

		for (auto it = nodes.begin(); it != nodes.end(); it++)
		{
			coords[nid * 3] = it->second.x;
			coords[nid * 3 + 1] = it->second.y;
			coords[nid * 3 + 2] = it->second.z;

			nidMap.insert(make_pair(it->first, nid));
			nidMapReverse.insert(make_pair(nid, it->first));

			nid++;
		}

		int eid = 0;

		for (auto it = elements.begin(); it != elements.end(); it++)
		{
			indices[eid * 4] = nidMap[it->second.n1];
			indices[eid * 4 + 1] = nidMap[it->second.n2];
			indices[eid * 4 + 2] = nidMap[it->second.n3];
			indices[eid * 4 + 3] = nidMap[it->second.n3];
			eid++;
		}

		KMAS::OnestepUnformSetMeshesEx(coords, nodes.size(), indices, elements.size(), 0, NULL, 0, NULL, 0, NULL, 0);
		KMAS::OnestepUnformSetSurfaceType(KMAS::MID_SURFACE);
		KMAS::TOnestepSolverErrorType ret;
		KMAS::OnestepUnformSolveEx(ret, false);

		if (ret == KMAS::SUCCESSFUL)
		{
			vector<double> flatten_shape;
			int fs_nlen = nodes.size();
			flatten_shape.resize(fs_nlen * 3);
			KMAS::OnestepUnformGetBlankShape(&flatten_shape[0], fs_nlen); //展平坐标	

			for (int i = 0; i < fs_nlen; i++)
			{
				Node _n;
				_n.nid = nidMapReverse[i];
				_n.x = flatten_shape[i * 3];
				_n.y = flatten_shape[i * 3 + 1];
				_n.z = flatten_shape[i * 3 + 2];
				flatNodes.insert(make_pair(_n.nid, _n));
			}
		}

		KMAS::OnestepUnformDestructor();

		GetOnestepCOG();

		return ret;
	}

	void Transform2ZPlane()
	{
		TopLoc_Location location;
		opencascade::handle<Poly_Triangulation> triFace = BRep_Tool::Triangulation(face, location);

		// 先找到两个点

		int n1, n2;

		for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());
			//TopLoc_Location location;

			//opencascade::handle<Poly_PolygonOnTriangulation> triEdge = BRep_Tool::PolygonOnTriangulation(objEdge, triFace, location);

			//if (triEdge.IsNull())
			//{
			//	continue;
			//}
			//edgesHash.insert(make_pair(hash<TopoDS_Edge>{}(objEdge), nodes));
			const TColStd_Array1OfInteger& nds = edgesHash[hash<TopoDS_Edge>{}(objEdge)];

			//const TColStd_Array1OfInteger& nds = triEdge->Nodes();

			//Transform2ZPlane(objEdge, nodes[nds[1]], nodes[nds.Size()], true);

			n1 = nds[1];
			n2 = nds.Size();

			if (n1 == n2)
			{
				n2 = nds.Size() - 1;
			}

			break;
		}

		gp_Vec v1(flatNodes[n2].x - flatNodes[n1].x, flatNodes[n2].y - flatNodes[n1].y, flatNodes[n2].z - flatNodes[n1].z);
		gp_Vec v2(cogOnestep.X() - flatNodes[n1].x, cogOnestep.Y() - flatNodes[n1].y, cogOnestep.Z() - flatNodes[n1].z);

		gp_Vec vo = v1 ^ v2;
		vo.Normalize();
		gp_Vec vz(0, 0, 1);
		gp_Vec vt;

		double angle;

		if (vo.IsEqual(vz, TOL, TOL))
		{
			angle = 0.0;

			for (auto it : flatNodes)
			{
				gp_Pnt2d tempNode(it.second.x, it.second.y);
				planeNodes.insert(make_pair(it.first, tempNode));
			}
		}
		else
		{
			if (vo.IsOpposite(vz, TOL))
			{
				angle = M_PI;
				vt.SetCoord(0.0, 1.0, 0.0);
			}
			else
			{
				angle = vo.Angle(vz);
				vt = vo ^ vz;
			}

			// 定义旋转轴
			gp_Pnt pt(flatNodes[n1].x, flatNodes[n1].y, flatNodes[n1].z);
			gp_Ax1 ax1(pt, vt);
			gp_Trsf tf;

			tf.SetRotation(ax1, angle);

			for (auto it = flatNodes.begin(); it != flatNodes.end(); it++)
			{
				gp_Pnt vn(it->second.x, it->second.y, it->second.z);
				gp_Pnt vnt = vn.Transformed(tf);
				planeNodes.insert(make_pair(it->first, gp_Pnt2d(vnt.X(), vnt.Y())));
			}
		}

		// 平移到原点
		gp_Pnt2d baseNode(planeNodes.begin()->second.X(), planeNodes.begin()->second.Y());
		for (auto it = planeNodes.begin(); it != planeNodes.end(); it++)
		{
			it->second.SetX(it->second.X() - baseNode.X());
			it->second.SetY(it->second.Y() - baseNode.Y());
		}

		jigsawNodes = planeNodes;

		GetPlaneCOG();

	}

	void Transform2ZPlane(const TopoDS_Edge &e, const Node &fatherStartNode, const Node& fatherEndNode, const FaceOrientation &fatherFaceOri)
	{
		// 这时的FlatNode是空间的不是Z平面上的
		if (flatNodes.size() == 0)
		{
			return;
		}

		int startNodeId, endNodeId;

		// 为了提高计算效率，下面dis都是实际距离的平方
		double disStart, disEnd;
		double disStartMin = 1e6;
		double disEndMin = 1e6;
		
		// 找到距离最近的点，因为有可能edge长度不一致；

		for (auto p : edges )
		{
			if (hash<TopoDS_Edge>{}(p.first) == hash<TopoDS_Edge>{}(e))
			{
				for (int i = 1; i <= edges[p.first].Size(); i++)
				{
					disStart = fatherStartNode.SquareDistanceTo(nodes[edges[p.first][i]]);
					disEnd = fatherEndNode.SquareDistanceTo(nodes[edges[p.first][i]]);

					if (disStart < disStartMin)
					{
						disStartMin = disStart;
						startNodeId = edges[p.first][i];
					}

					if (disEnd < disEndMin)
					{
						disEndMin = disEnd;
						endNodeId = edges[p.first][i];
					}
				}
				break;
			}
		}

		// 为了保持与父面展开法线一致，选择对调的端点后计算向量	

		jigsawStartNodeId = startNodeId;
		jigsawEndNodeId = endNodeId;

		int n1 = startNodeId;
		int n2 = endNodeId;

		if (fabs(flatNodes[n1].DistanceTo(flatNodes[n2]) - 36) < 1)
		{
			int a = 1;
		}

		// 检查变换方向是否OK
		// 先按照此方向将三个点变换到Z平面，之后计算二维的叉乘结果，因为三维叉乘不能判断角度的正负

		FaceOrientation faceOri = GetOrientation(startNodeId, endNodeId);

		gp_Vec v1(flatNodes[n2].x - flatNodes[n1].x, flatNodes[n2].y - flatNodes[n1].y, flatNodes[n2].z - flatNodes[n1].z);
		gp_Vec v2;
		gp_Vec vo;

		// 找到一个距离最远的点，并且不在一条直线上，构成向量
		double dis3Max = -1e6;

		for (set<int>::iterator it = edgeNodes.begin(); it != edgeNodes.end(); it++)
		{

			TColStd_Array1OfInteger edgsNd = edgesHash[hash<TopoDS_Edge>{}(e)];

			bool edgeNodeFlag = false;

			// 判断点是否是基准edge上的点，如果是进入下一个循环
			for (int i = 1; i <= edgesHash[hash<TopoDS_Edge>{}(e)].Size(); i++)
			{
				if (*it == edgesHash[hash<TopoDS_Edge>{}(e)][i])
				{
					edgeNodeFlag = true;
					break;
				}
			}

			if (edgeNodeFlag)
			{
				continue;
			}

			double dis = nodes[*it].SquareDistanceTo(nodes[n1]);
			if (dis > dis3Max)
			{	
				v2.SetXYZ(gp_XYZ(flatNodes[*it].x - flatNodes[n1].x, flatNodes[*it].y - flatNodes[n1].y, flatNodes[*it].z - flatNodes[n1].z));			

				if (v1.IsParallel(v2, 1e-6))
				{
					continue;
				}

				dis3Max = dis;	
			}
		}

		vo = v1 ^ v2;

		vo.Normalize();

		gp_Vec vz(0, 0, 1);
		gp_Vec vt;

		if (faceOri == fatherFaceOri)
		{
			vz.SetZ(-1.0);
		}
		
		double angle;

		if (vo.IsEqual(vz, TOL, TOL))
		{
			angle = 0.0;

			for (auto it : flatNodes)
			{
				gp_Pnt2d tempNode(it.second.x, it.second.y);
				planeNodes.insert(make_pair(it.first, tempNode));
			}
		}
		else
		{
			if (vo.IsOpposite(vz, TOL))
			{
				angle = M_PI;
				vt.SetCoord(0.0, 1.0, 0.0);
			}
			else
			{
				angle = vo.Angle(vz);
				vt = vo ^ vz;
			}

			// 定义旋转轴
			gp_Pnt pt(flatNodes[n1].x, flatNodes[n1].y, flatNodes[n1].z);
			gp_Ax1 ax1(pt, vt);
			gp_Trsf tf;

			tf.SetRotation(ax1, angle);

			for (auto it = flatNodes.begin(); it != flatNodes.end(); it++)
			{
				gp_Pnt vn(it->second.x, it->second.y, it->second.z);
				gp_Pnt vnt = vn.Transformed(tf);
				planeNodes.insert(make_pair(it->first, gp_Pnt2d(vnt.X(), vnt.Y())));
			}
		}

		GetPlaneCOG();
	}

	FaceOrientation GetOrientation(const int& n1, const int& n2)
	{
		gp_Vec v1(flatNodes[n2].x - flatNodes[n1].x, flatNodes[n2].y - flatNodes[n1].y, flatNodes[n2].z - flatNodes[n1].z);
		gp_Vec v2(cogOnestep.X() - flatNodes[n1].x, cogOnestep.Y() - flatNodes[n1].y, cogOnestep.Z() - flatNodes[n1].z);
		gp_Vec vo = v1 ^ v2;

		vo.Normalize();

		gp_Vec vt(0, 0, 1);
		gp_Vec vRef(1, 0, 0);

		gp_Mat trsMatrix = RodriguesMatrix(vo, vt, vRef);

		gp_XYZ vn1(flatNodes[n1].x, flatNodes[n1].y, flatNodes[n1].z);
		gp_XYZ vn1t = vn1 * trsMatrix;
		gp_Pnt2d pt1(vn1t.X(), vn1t.Y());

		gp_XYZ vn2(flatNodes[n2].x, flatNodes[n2].y, flatNodes[n2].z);
		gp_XYZ vn2t = vn2 * trsMatrix;
		gp_Pnt2d pt2(vn2t.X(), vn2t.Y());

		gp_XYZ vnCog(cogOnestep.X(), cogOnestep.Y(), cogOnestep.Z());
		gp_XYZ vnCogt = vnCog * trsMatrix;
		gp_Pnt2d ptCog(vnCogt.X(), vnCogt.Y());

		gp_Vec2d vt1(pt2.X() - pt1.X(), pt2.Y() - pt1.Y());
		gp_Vec2d vt2(ptCog.X() - pt1.X(), ptCog.Y() - pt1.Y());

		vt1.Normalize();
		vt2.Normalize();

		if (vt1.Crossed(vt2) > 0)
		{
			return Anticlockwise;
		}
		else
		{
			return Clockwise;
		}
	}

	FaceOrientation GetPlaneOrientation(const int& n1, const int& n2)
	{
		gp_Vec v1(planeNodes[n2].X() - planeNodes[n1].X(), planeNodes[n2].Y() - planeNodes[n1].Y(), 0);
		gp_Vec v2(cogPlane.X() - planeNodes[n1].X(), cogPlane.Y() - planeNodes[n1].Y(), 0);
		gp_Vec vo = v1 ^ v2;

		vo.Normalize();

		if (vo.Z() > 0)
		{
			return Anticlockwise;
		}
		else
		{
			return Clockwise;
		}
	}


	gp_Mat RodriguesMatrix(gp_Vec v1, gp_Vec v2, gp_Vec vRef)
	{
		//v1: origin,  v2 transformed.
		//vRef: 这个向量的作用是，一旦两个向量平行但方向相反时，则以这个向量为法向进行旋转
		//先计算向量平行的情况，因为平行时计算出的叉乘值为0

		double PI = 3.14159265;


		gp_Vec crossResult;

		double theta;

		if (fabs(v1.Angle(v2)) < TOL)
		{
			gp_Mat matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
			return matrix;
		}

		if (fabs(PI - fabs(v1.Angle(v2))) < TOL)
		{
			crossResult = vRef;
			theta = PI;
		}
		else
		{
			crossResult = v1 ^ v2;
			double norm = sqrt(crossResult.X() * crossResult.X() + crossResult.Y() * crossResult.Y() + crossResult.Z() * crossResult.Z());
			theta = atan2(norm, v1 * v2);
			crossResult.Normalize();
		}


		//cout << "theta: " << theta/3.1416 * 180 << endl;

		//https://wuli.wiki/changed/RotA.html

		double A11, A12, A13, A21, A22, A23, A31, A32, A33;
		double c = cos(theta);
		double s = sin(theta);
		double a = 1 - c;

		double Ax = crossResult.X();
		double Ay = crossResult.Y();
		double Az = crossResult.Z();

		A11 = a * Ax * Ax + c;
		A12 = a * Ax * Ay - s * Az;
		A13 = a * Ax * Az + s * Ay;

		A21 = a * Ay * Ax + s * Az;
		A22 = a * Ay * Ay + c;
		A23 = a * Ay * Az - s * Ax;

		A31 = a * Az * Ax - s * Ay;
		A32 = a * Az * Ay + s * Ax;
		A33 = a * Az * Az + c;

		gp_Mat matrix(A11, A12, A13, A21, A22, A23, A31, A32, A33);

		matrix.Transpose();

		return matrix;
	}

	void Jigsaw(const gp_Pnt2d& fatherStartNode, const gp_Pnt2d& fatherEndNode)
	{
		cout << "Jigsaw..." << endl;
		gp_Vec2d fatherVec(fatherStartNode, fatherEndNode);
		gp_Vec2d vec(planeNodes[jigsawStartNodeId], planeNodes[jigsawEndNodeId]);

		double transAngle = vec.Angle(fatherVec);

		for (MAP<int, gp_Pnt2d>::iterator it = planeNodes.begin(); it != planeNodes.end(); it++)
		{
			gp_Pnt2d pt = it->second.Translated(planeNodes[jigsawStartNodeId], fatherStartNode);
			pt.Rotate(fatherStartNode, transAngle);
			jigsawNodes.insert(make_pair(it->first, pt));
		}
	}

	static void BuildDocStd(const TopoDS_Shape& shape)
	{
		Handle(TDocStd_Application) app = new TDocStd_Application;
		Handle(TDocStd_Document) doc;
		app->NewDocument("BinXCAF", doc);

		TDF_Label mainLab = doc->Main();
		Handle(XCAFDoc_ShapeTool)ST = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

		ST->AddShape(shape);

		ModelIO modelio;
		//modelio.Write("F:\\test.stl", doc, GeomSTL);
		modelio.Write("D:\\1_edge.stp", doc, GeomSTP);

		app->Close(doc);
	}

	void CreateOutline(TopoDS_Compound &aCompound , FlattenFace & flat , map<TopoDS_Edge, TopoDS_Edge > &NewMapEdgeAdjEdge)
	{
		SaveDxfFile saveFile;
		double offset = M_PI;
		string str = "0";
		BRep_Builder aBuilder;
		for (auto item : edges)	// MAP<TopoDS_Edge, TColStd_Array1OfInteger>edges			
		{		
			set<TopoDS_Edge> setEdges;
			Handle(TColgp_HArray1OfPnt2d) aPoints = new TColgp_HArray1OfPnt2d(1, item.second.Size());

			for (int i = 1; i <= item.second.Size(); i++)
			{
				int nid = item.second[i];				
				aPoints->ChangeValue(i) = jigsawNodes[nid];
			}

			SheetFlattenFitting* sheetFlatFit = new SheetFlattenFitting();
			FitCurve fitCurve;

			if (sheetFlatFit->Perform(aPoints, fitCurve) >= 0)
			{
				/*TopoDS_Edge edges;
				if (NewMapEdgeAdjEdge.find(item.first) != NewMapEdgeAdjEdge.end())
				{
					edges = NewMapEdgeAdjEdge.find(item.first)->second;
				}
				aBuilder.Add(aCompound, fitCurve.edge);
				NewMapEdgeAdjEdge[item.first] = edges;*/
				NewMapEdgeAdjEdge[item.first] = fitCurve.edge;
			}
			gp_Pnt2d p1 = std::get<0>(fitCurve.line);
			gp_Pnt2d p2 = std::get<1>(fitCurve.line);
			point_t point1(p1.X(), p1.Y());
			point_t point2(p2.X(), p2.Y());
			flat.ai.emplace_back(point1, point2, str);
			point_t point3(std::get<0>(fitCurve.circle).X(), std::get<0>(fitCurve.circle).Y());
			//flat.ai1.emplace_back(point3, std::get<1>(fitCurve.circle), "0");
			flat.ai2.emplace_back(point3, std::get<1>(fitCurve.circle), (std::get<2>(fitCurve.circle)+2*offset)*(180.0/offset), (std::get<3>(fitCurve.circle)+2*offset) * (180.0 / offset), "0");
			point_t point4(std::get<0>(fitCurve.eclipse).X(), std::get<0>(fitCurve.eclipse).Y());
			//flat.ai2.emplace_back(point4, std::get<1>(fitCurve.eclipse), std::get<2>(fitCurve.eclipse), std::get<3>(fitCurve.eclipse), "0");
			//saveFile.SaveDxf(ai, ai1, ai2, "D:\\1_work\\test.dxf");
			delete sheetFlatFit;
		}
		saveFile.SaveDxf(flat.ai, flat.ai1, flat.ai2, "D:\\1_work\\test.dxf");
	}

	bool IsLine(TColStd_Array1OfInteger aEdge)
	{
		if (aEdge.Size() == 2)
		{
			return true;
		}

		int startPointId = aEdge[1];
		int endPointId = aEdge[aEdge.Size()];
		double dis = jigsawNodes[startPointId].Distance(jigsawNodes[endPointId]);

		for (int i = 2; i < aEdge.Size() - 1; i++)
		{
			if (!isOnLine(jigsawNodes[startPointId], jigsawNodes[endPointId], jigsawNodes[aEdge[i]], dis))
			{
				return false;
			}
		}
		return true;
	}

	// 只适用于测试点在端点内部的情况
	bool isOnLine(const gp_Pnt2d& startPoint, const gp_Pnt2d& endPoint, const gp_Pnt2d& checkPoint, const double& dis)
	{
		if (fabs(startPoint.Distance(checkPoint) + endPoint.Distance(checkPoint) - dis) < 1e-6)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void DumpNasEx(string nasFile, int flag = 0)
	{

		auto getEightWidth = [](const double& num)
		{
			//float _num = float(num);
			//stringstream ss;
			string str;
			//ss << num;
			//ss >> str;
			str = to_string(num);
			if (str.length() <= 16)
			{
				return str;
			}
			else
			{
				return str.substr(0, 16);
			}
		};

		fstream outfile;
		outfile.open(nasFile.c_str(), ios::out);

		if (flag == 0)
		{
			for (auto it = nodes.begin(); it != nodes.end(); it++)
			{
				outfile << "GRID*   ";
				outfile << setw(16) << it->first;
				outfile << "                ";
				outfile << setw(16) << getEightWidth(it->second.x);
				outfile << setw(16) << getEightWidth(it->second.y);
				outfile << endl;
				outfile << "*       ";
				outfile << setw(16) << getEightWidth(it->second.z);
				outfile << endl;
			}
		}
		else if (flag == 1)
		{
			for (auto it = flatNodes.begin(); it != flatNodes.end(); it++)
			{
				outfile << "GRID*   ";
				outfile << setw(16) << it->first;
				outfile << "                ";
				outfile << setw(16) << getEightWidth(it->second.x);
				outfile << setw(16) << getEightWidth(it->second.y);
				outfile << endl;
				outfile << "*       ";
				outfile << setw(16) << getEightWidth(it->second.z);
				outfile << endl;
			}
		}
		else if (flag == 2)
		{
			for (auto it = planeNodes.begin(); it != planeNodes.end(); it++)
			{
				outfile << "GRID*   ";
				outfile << setw(16) << it->first;
				outfile << "                ";
				outfile << setw(16) << getEightWidth(it->second.X());
				outfile << setw(16) << getEightWidth(it->second.Y());
				outfile << endl;
				outfile << "*       ";
				outfile << setw(16) << 0.0;
				outfile << endl;
			}
		}
		else if (flag == 3)
		{
			for (auto it = jigsawNodes.begin(); it != jigsawNodes.end(); it++)
			{
				outfile << "GRID*   ";
				outfile << setw(16) << it->first;
				outfile << "                ";
				outfile << setw(16) << getEightWidth(it->second.X());
				outfile << setw(16) << getEightWidth(it->second.Y());
				outfile << endl;
				outfile << "*       ";
				outfile << setw(16) << 0.0;
				outfile << endl;
			}
		}

		int barId = 0;

		set<int> pids;

		for (auto it = elements.begin(); it != elements.end(); it++)
		{
			outfile << "CTRIA3* ";
			outfile << setw(16) << it->first;
			outfile << setw(16) << it->second.pid;
			outfile << setw(16) << it->second.n1;
			outfile << setw(16) << it->second.n2;
			outfile << endl;
			outfile << "*       ";
			outfile << setw(16) << it->second.n3;
			outfile << endl;

			if (it->first > barId)
			{
				barId = it->first;
			}

			pids.insert(it->second.pid);
		}

		// PSHELL         1       1      1.       1      1.       1
		for (auto it = pids.begin(); it != pids.end(); it++)
		{
			outfile << "PSHELL* ";
			outfile << setw(16) << *it;
			outfile << "               1              1.               1" << endl;
			outfile << "*                     1.               1";
			outfile << endl;
		}

		outfile << "MAT1*                  1         210000.                             0.3" << endl;;
		outfile << "* 7.85E-9" << endl;
		outfile.close();
	}

	void Release()
	{
		nodes.clear();
		flatNodes.clear();
		elements.clear();
		edges.clear();
	}
};

// 重载TopoDS_Edge和TopoDS_Face运算法

bool operator <(const TopoDS_Edge& e1, const TopoDS_Edge& e2)
{
	if (hash<TopoDS_Edge>{}(e1) < hash<TopoDS_Edge>{}(e2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool operator <(const TopoDS_Face& f1, const TopoDS_Face& f2)
{
	if (hash<TopoDS_Face>{}(f1) < hash<TopoDS_Face>{}(f2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

struct FlattenFaceNode
{
	TopoDS_Edge edge;
	FlattenFace flattenFace;
	int startNodeId;
	int endNodeId;
	Node startNode;
	Node endNode;
	FaceOrientation faceOri;

	int id;

	size_t edgeHash;
	size_t faceHash;
	vector<size_t> faceEdgeHash;

	void InitHash()
	{
		faceEdgeHash.clear();
		for (TopExp_Explorer edgeExp(flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

			faceEdgeHash.push_back(hash<TopoDS_Edge>{}(objEdge));

		}

		edgeHash = hash<TopoDS_Edge>{}(edge);
		faceHash = hash<TopoDS_Face>{}(flattenFace.face);
	}


	bool isNull()
	{
		if (flattenFace.face.IsNull())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool isRoot()
	{
		if (edge.IsNull())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void ParseNode(const TopoDS_Edge& objEdge)
	{
		TopLoc_Location location;
		opencascade::handle<Poly_Triangulation> triFace = BRep_Tool::Triangulation(flattenFace.face, location);
		opencascade::handle<Poly_PolygonOnTriangulation> triEdge;

		for (TopExp_Explorer edgeExp(flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& _edge = TopoDS::Edge(edgeExp.Current());

			if (hash<TopoDS_Edge>{}(objEdge) == hash<TopoDS_Edge>{}(_edge))
			{
				triEdge = BRep_Tool::PolygonOnTriangulation(_edge, triFace, location);
				break;
			}
		}

		const TColStd_Array1OfInteger& nds = triEdge->Nodes();

		startNodeId = nds[1];
		startNode = flattenFace.GetNode(nds[1]);

		if (nds[1] == nds[nds.Size()])
		{
			endNodeId = nds[nds.Size() - 1];
			endNode = flattenFace.GetNode(nds[nds.Size() - 1]);
		}
		else
		{
			endNodeId = nds[nds.Size()];
			endNode = flattenFace.GetNode(nds[nds.Size()]);
		}

		// 这里是获取父平面的方向，应该按照Transform后的计算
		faceOri = flattenFace.GetPlaneOrientation(startNodeId, endNodeId);
	}

	FaceOrientation GetPlaneOrientation()
	{
		return faceOri;
	}

	Node StartNode()
	{
		return flattenFace.GetNode(startNodeId);
	}

	Node EndNode()
	{
		return flattenFace.GetNode(endNodeId);
	}

	Node StartNodeFlat()
	{
		return flattenFace.GetFlatNode(startNodeId);
	}

	Node EndNodeFlat()
	{
		return flattenFace.GetFlatNode(endNodeId);
	}

	gp_Pnt2d StartNode2D()
	{
		return flattenFace.GetJigsawNode(startNodeId);
	}

	gp_Pnt2d EndNode2D()
	{
		return flattenFace.GetJigsawNode(endNodeId);
	}

	friend bool operator < (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace.face < rhs.flattenFace.face;
	}

	friend bool operator == (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace.face == rhs.flattenFace.face;
	}
};