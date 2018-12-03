// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include <queue> //To use the priority queue
#include <iostream>

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

// I create a struct to represent the positions of the Cells in the map and the value of it.
struct Cell {
	int x, y, z;
	float value;
	Cell(int xx=0, int yy=0, int zz = 0, float v=0) :x(xx), y(yy), z(zz), value(v) {};
};

//Auxiliar functions to convert a position to a cell and a cell to a center center position.
Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeigth) {
	return Vector3((j*cellWidth) + cellWidth * 0.5f, (i*cellHeigth) + cellHeigth * 0.5f, 0);
}

Cell positionToCell(const Vector3 pos, float cellWidth, float cellHeight) {
	return Cell((int)(pos.x*1.0f / cellWidth), (int)(pos.y*1.0f / cellHeight));
}




//This function evaluate a cell for the Extraction Center, taking account the obstacles near the cell. If the defenses were better to put together, 
//it possible use this function to place them, changing obstacles  by extraction center and defenses placed.
float cellValueMain(int col, int row, float r, List<Object*> obstacles, int nCw, int nCh, float mW, float mH) {
	float v = 0, d;
	Vector3 pos((col / (float)nCw)*mW, (row / (float)nCh)*mH, 0);
	std::list<Object*>::const_iterator cObs = obstacles.begin();
	//std::cout << "Start the loop IN cellValueMain" << std::endl;
	while (cObs != obstacles.end()) {
		//std::cout << "Calculate the distance" << std::endl;
		d = _distance((*cObs)->position, pos);
		if (d >= 2 * r) {
		//	std::cout << "Entering in the IF" << std::endl;
			v += (d + (*cObs)->radio) / (*cObs)->radio;
		}
		++cObs;

	}
	return v;
	
}

//This function evaluate a cell for the rest of Defenses. The value depend of the distance to the center.
float cellValueDef(int cCol, int cRow, int col, int row) {
	//std::cout << "In the function cellValueDef" << std::endl;
	//Vector3 pCell = cellCenterToPosition(row, col, cellWidth, cellHeight);
	return sqrt(pow(cCol-col,2) + pow(cRow-row,2));
}

//To use priority_queue (an structure of STL) and the function sort (of STL too), it necessary to define the comparison operator < and >.
bool operator >(const Cell& c1, const Cell& c2) {
	return c1.value > c2.value;
}
bool operator <(const Cell& c1, const Cell& c2) {
	return c1.value < c2.value;
}

/*To compare if 2 positions are equal:
bool operator == (Vector3 p1, Vector3 p2) {
	return p1.x == p2.x && p1.y == p2.y;
}
bool operator != (Vector3 p1, Vector3 p2) {
	return !(p1 == p2);
}*/

//To determine if it is possible these positioning.
bool factible(Defense* def, const std::list<Defense*>& placed, const std::list<Object*>& obs, float mH, float mW) {
	//Does the Defense exceed the maps dimensions? 
	bool fac = def->position.x + def->radio <= mW && def->position.x - def->radio >= 0 &&
		def->position.y + def->radio <= mH && def->position.y - def->radio >= 0;
	//Does the Defense crash into another Defense?
	std::list<Defense*>::const_iterator it_c = placed.begin();
	while (fac && it_c != placed.end()) {
		fac = _distance(def->position, (*it_c)->position) >= def->radio + (*it_c)->radio;
		++it_c;
	}
	//Does the Defense crash into an obstacle?
	std::list<Object*>::const_iterator it_o = obs.begin();
	while (fac && it_o != obs.end()) {
		fac = _distance(def->position, (*it_o)->position) >= def->radio + (*it_o)->radio;
		++it_o;
	}

	return fac;
}

//To sort the Defenses, i overload the operator > to use the function sort of the list.
bool operatorDefSort (Defense* d1, Defense* d2) {
	//First the biggest range.
	if (d1->range > d2->range) {
		return true;
	}else {
		if (d1->range == d2->range) {
			//Then dispersion factor for damage.
			if (d1->dispersion / d1->damage > d2->dispersion / d2->damage) {
				return true;
			}else {
				if (d1->dispersion / d1->damage == d2->dispersion / d2->damage) {
					//then the biggest dispersion.
					if (d1->dispersion > d2->dispersion) {
						return true;
					}else {
						if (d1->dispersion == d2->dispersion) {
							//finaly the biggest damage.
							if (d1->damage > d2->damage) {
								return true;
							}else {
								return false;
							}
						}else {
							return false;
						}
					}
				}else {
					return false;
				}
			}
		}else {
			return false;
		}
	}
}

void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
	, std::list<Object*> obstacles, std::list<Defense*> defenses) {

	float cellWidth = mapWidth / nCellsWidth;
	float cellHeight = mapHeight / nCellsHeight;

	std::list<Defense*>::iterator cDef = defenses.begin();
	std::list<Defense*> placed;
	Cell cCell;
	//Vector3 pos;
	int centerCol = 0, centerRow = 0;
	std::priority_queue<Cell> mainCells;
	//std::cout << "Start the loop to cellValue" << std::endl;
	//Giving value to the cells to place the Extraction Center.
	for (int x = 0; x < nCellsWidth; x++) {
		for (int y = 0; y < nCellsHeight; y++) {
			if (freeCells[x][y]) {
				/*pos = cellCenterToPosition(y, x, cellWidth, cellHeight);
				cCell.x = pos.x;
				cCell.y = pos.y;
				cCell.value = cellValueMain(pos, cellWidth, cellHeight, mapWidth, mapHeight, obstacles, (*cDef)->radio*5);*/
				mainCells.push(Cell(x, y, cellValueMain(x, y, (*defenses.begin())->radio*5, obstacles, nCellsWidth, nCellsHeight, mapWidth, mapHeight)));
				//std::cout << "try to place Extraction center" << std::endl; 
			}
		}
	}
	
	//When all the cells have a value, it is turn to check if is possible place the Extraction Center in it.
	bool p = false;
	while (!mainCells.empty() && !p) {
		cCell = mainCells.top();
		mainCells.pop();
		(*cDef)->position.x = cCell.x * cellWidth + cellWidth * 0.5f;
		(*cDef)->position.y = cCell.y * cellHeight + cellHeight * 0.5f;
		(*cDef)->position.z = 0;
		if (factible(*cDef, placed, obstacles, mapHeight, mapWidth)) {
			p = true;
			centerCol = cCell.x;
			centerRow = cCell.y;
			placed.push_back(*cDef);
			++cDef;
		}
	}
	defenses.pop_front();
	defenses.sort(operatorDefSort);

	cDef = defenses.begin();
	std::vector<Cell> defCells;
	//std::cout << "Start the loop to cellValue of the rest cells" << std::endl;
	//Now turn to calculate the value of the cells for the rest of the defenses.
	for (int x = 0; x < nCellsWidth; x++) {
		for (int y = 0; y < nCellsHeight; y++) {
			if (freeCells[x][y]) {
				//std::cout << "Entering in the two FOR" << std::endl;
				defCells.push_back(Cell(x, y, cellValueDef(centerCol, centerRow, x, y)));
			}
		}
	}
	//defCells.sort();
	std::sort(defCells.begin(), defCells.end(), std::less<Cell>());

	
	/*std::list<Cell>::iterator cellC = defCells.begin();
	std::list<Cell>::iterator cellCaux;*/
	int iterMax = cellWidth * cellHeight*defenses.size(), iter = 0, i = 0;
	//Now check where is possible to place the defenses.
	while (!defCells.empty() && iter++ < iterMax && cDef != defenses.end()) {
		
			cCell = defCells[i];
			(*cDef)->position.x = cCell.x * cellWidth + cellWidth * 0.5f;
			(*cDef)->position.y = cCell.y * cellHeight + cellHeight * 0.5f;
			(*cDef)->position.z = 0;
			if (factible(*cDef, placed, obstacles, mapHeight, mapWidth)) {
				placed.push_back(*cDef);
				defCells.erase(defCells.begin()+i);
				++cDef;
				std::cout << "placed" << std::endl;
			}
			if (i == defCells.size()) {
				i = 0;
			}else {
				++i;
			}
		
	}




	







	/*
    int maxAttemps = 1000;
    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end() && maxAttemps > 0) {

        (*currentDefense)->position.x = ((int)(_RAND2(nCellsWidth))) * cellWidth + cellWidth * 0.5f;
        (*currentDefense)->position.y = ((int)(_RAND2(nCellsHeight))) * cellHeight + cellHeight * 0.5f;
        (*currentDefense)->position.z = 0; 
        ++currentDefense;
    }*/

#ifdef PRINT_DEFENSE_STRATEGY

    float** cellValues = new float* [nCellsHeight]; 
    for(int i = 0; i < nCellsHeight; ++i) {
       cellValues[i] = new float[nCellsWidth];
       for(int j = 0; j < nCellsWidth; ++j) {
           cellValues[i][j] = ((int)(cellValue(i, j))) % 256;
       }
    }
    dPrintMap("strategy.ppm", nCellsHeight, nCellsWidth, cellHeight, cellWidth, freeCells
                         , cellValues, std::list<Defense*>(), true);

    for(int i = 0; i < nCellsHeight ; ++i)
        delete [] cellValues[i];
	delete [] cellValues;
	cellValues = NULL;

#endif
}
