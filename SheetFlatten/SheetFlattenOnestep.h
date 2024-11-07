#pragma once

#include "SheetFlattenBase.h"


#define BUILD_OUTSIDE_NX_ENV
#include "OneStep/BodyDes_OnestepUnformSolver.hxx"


class SheetFlattenOneStep
{
public:
	int Perform(FlattenFace& flattenFace);
	void ExportNas(const string filePath);
private:
	FlattenFace myFlattenFace;
};

inline int SheetFlattenOneStep::Perform(FlattenFace& flattenFace)
{
	KMAS::OnestepUnformConstructor(KMAS::WHOLE_UNFOLD);

	double* coords = new double[flattenFace.nodes.size() * 3];
	int* elements = new int[flattenFace.elements.size() * 4];

	map<int, int> nidMap;
	map<int, int> nidMapReverse;
	int nid = 0;
	for (auto it = flattenFace.nodes.begin(); it != flattenFace.nodes.end(); it++)
	{
		coords[nid * 3] = it->second.x;
		coords[nid * 3 + 1] = it->second.y;
		coords[nid * 3 + 2] = it->second.z;

		nidMap.insert(make_pair(it->first, nid));
		nidMapReverse.insert(make_pair(nid, it->first));

		nid++;
	}

	int eid = 0;

	for (auto it = flattenFace.elements.begin(); it != flattenFace.elements.end(); it++)
	{
		elements[eid * 4] = nidMap[it->second.n1];
		elements[eid * 4 + 1] = nidMap[it->second.n2];
		elements[eid * 4 + 2] = nidMap[it->second.n3];
		elements[eid * 4 + 3] = nidMap[it->second.n3];
		eid++;
	}

	KMAS::OnestepUnformSetMeshesEx(coords, flattenFace.nodes.size(), elements, flattenFace.elements.size(), 0, NULL, 0, NULL, 0, NULL, 0);
	KMAS::OnestepUnformSetSurfaceType(KMAS::MID_SURFACE);
	KMAS::TOnestepSolverErrorType ret;
	KMAS::OnestepUnformSolveEx(ret, false);

	if (ret == KMAS::SUCCESSFUL)
	{
		vector<double> flatten_shape;
		int fs_nlen = flattenFace.nodes.size();
		flatten_shape.resize(fs_nlen * 3);
		KMAS::OnestepUnformGetBlankShape(&flatten_shape[0], fs_nlen); //Õ¹Æ½×ø±ê	

		for (int i = 0; i < fs_nlen; i++)
		{
			Node _n;
			_n.nid = nidMapReverse[i];
			_n.x = flatten_shape[i * 3];
			_n.y = flatten_shape[i * 3 + 1];
			_n.z = flatten_shape[i * 3 + 2];
			flattenFace.flatNodes.insert(make_pair(_n.nid, _n));
		}
	}

	KMAS::OnestepUnformDestructor();

	myFlattenFace = flattenFace;

	return ret;
}

inline void SheetFlattenOneStep::ExportNas(const string filePath)
{
	myFlattenFace.DumpNasEx(filePath, 1);
}