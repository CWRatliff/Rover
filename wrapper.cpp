#include "Sapphire.h"
#include <cstddef>
#include <cstdio>

Sapphire 	dbase;
RDbf*		db;

extern "C" int Cdblogin() {
	db=dbase.DbLogin("robodatabase");
	if (db == NULL)
		return 0;
	return (0);
	}

extern "C" int Cdblogout() {
	dbase.DbLogout(db);
	return (0);
	}

extern "C" int CFirst(void* tab, void* ndx) {
	RIndex* index;
	RTable* table;
	int		rc;
	
	table = (RTable*)tab;
	index = (RIndex*)ndx;
	rc = table->DbFirstRecord(index);
	return (rc);
	}

extern "C" int CNext(void* tab, void* ndx) {
	RIndex* index;
	RTable* table;
	int		rc;
	
	table = (RTable*)tab;
	index = (RIndex*)ndx;
	rc = table->DbNextRecord(index);
	return (rc);
	}

extern "C" void* COpenTable(char* tab) {
	RTable* CRel;
	CRel = db->DbOpenTable(tab);
	return (void *)CRel;
	}
	
extern "C" void* COpenIndex(void* tab, char* ndxname) {
	RIndex* ndx;
	RTable* table;
	
	table = (RTable*)tab;
	ndx = table->DbGetIndexObject(ndxname);
	return (void *)ndx;
	}

extern "C" int CSearchI(void* tab, void* ndx, int i1) {
	RIndex* index;
	RTable* table;
	char 	key[20];
	int		rc;
	
	table = (RTable*)tab;
	index = (RIndex*)ndx;
	MakeSearchKey(key, "i", i1);
	rc = table->DbSearchRecord(index, key);
	return (rc);
	}
	
extern "C" int CSearchII(void* tab, void* ndx, int i1, int i2) {
	RIndex* index;
	RTable* table;
	char 	key[20];
	int		rc;
	
	table = (RTable*)tab;
	index = (RIndex*)ndx;
	MakeSearchKey(key, "ii", i1, i2);
	rc = table->DbSearchRecord(index, key);
	return (rc);
	}

extern "C" int CSearchS(void* tab, void* ndx, char* s) {
	RIndex* index;
	RTable* table;
	char 	key[20];
	int		rc;
	
	table = (RTable*)tab;
	index = (RIndex*)ndx;
	MakeSearchKey(key, "i", s);
	rc = table->DbSearchRecord(index, key);
	return (rc);
	}
	
extern "C" const char* CGetCharPtr(void* tab, char* fld) {
	RTable* table;
	const char *p;
	
	table = (RTable*)tab;
	p = table->DbGetChar(fld);
	return (p);
	}

extern "C" double CGetDouble(void* tab, char* fname) {
	RTable* table;
	double	dnumber;

	table = (RTable*)tab;
	dnumber = table->DbGetDouble(fname);
	return (dnumber);
	}
	
	
