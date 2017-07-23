/*
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "m3rt/base/toolbox.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdarg.h>
#include <cstdlib>
#include <time.h>
#include <ios>
#include <ctime>
#include <sstream>
#include <map>
namespace m3rt
{
using namespace std;

void M3_WARN(const char *format, ...)
{
    //FILE * pFile = NULL;
    string path;

    /*	if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
        {
            path.append(LOG_FILE);
            pFile = fopen (path.c_str(),"a");
        }	*/

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    printf("M3 WARNING: ");
    printf("%s", buffer);
    /*	 if (pFile)
         {
            fprintf(pFile,"M3 WARNING: ");
            fprintf(pFile,"%s", buffer);
            fclose (pFile);
         }*/
    va_end(args);
}

void M3_ERR(const char *format, ...)
{
    FILE *pFile = NULL;
    string path;

    /*if (GetEnvironmentVariable(M3_ROBOT_ENV_VAR, path))
    {
        path.append(LOG_FILE);
        pFile = fopen (path.c_str(),"a");
    }*/

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    printf("M3 ERROR: ");
    printf("%s", buffer);
    if(pFile) {
        fprintf(pFile, "M3 ERROR: ");
        fprintf(pFile, "%s", buffer);
        fclose(pFile);
    }
    va_end(args);
}

void M3_DEBUG(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    printf("M3 DEBUG: ");
    printf("%s", buffer);
    va_end(args);
}

void M3_INFO(const char *format, ...)
{
    //FILE * pFile = NULL;
    string path;

    /*	if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
        {
            path.append(LOG_FILE);
            pFile = fopen (path.c_str(),"a");
        }	*/

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    printf("M3 INFO: ");
    printf("%s", buffer);
    /*	 if (pFile)
         {
            fprintf(pFile,"M3 INFO: ");
            fprintf(pFile,"%s", buffer);
            fclose (pFile);
         }*/
    va_end(args);
}

void BannerPrint(int width, const char *format, ...)
{
    char buf[81];
    width = width > 80 ? 80 : width;
    char out[] = "--------------------------------------------------------------------------------"; //len 80
    va_list args;
    va_start(args, format);
    snprintf(buf, 80, format, args);
    M3_PRINTF("\n");
    int n = (int)strlen(buf);
    if(n < width && ((width - n) / 2) > 0) {
        out[(width - n) / 2] = 0;
        M3_PRINTF("%s", out);
        M3_PRINTF("%s", buf);
        M3_PRINTF("%s", out);
    } else
        M3_PRINTF("%s", buf);
    va_end(args);
    M3_PRINTF("\n");
}

bool GetEnvironmentVariable(const char *var, vector<string>& result)
{
    //static vector<string> result;
    result.clear();
    if(!result.empty())
        return false;
    char const* tmp = getenv(var);
    if ( tmp == NULL ) {
        M3_ERR("%s is not set, exiting.\n",var);
        return false;
    }
    std::string PATH(tmp);
    const char delimiter = ':';

    if(PATH.empty())
        return false;

    size_t previous = 0;
    size_t index = PATH.find(delimiter);
    while(index != string::npos) {
        const std::string curr = PATH.substr(previous, index - previous);
        if(! ContainsString(result,curr))
            result.push_back(curr);
        previous = index + 1;
        index = PATH.find(delimiter, previous);
    }
    const std::string last = PATH.substr(previous);
    if(! ContainsString(result,last))
        result.push_back(last);
    return true;
}

bool GetEnvironmentVar(const char *var, string &s)
{
    char *p = getenv(var);
    if(p != NULL) {
        s.assign(p);
        return true;
    }
    return false;
}
vector<mReal> YamlReadVectorM(string s)
{
    size_t start = s.find_first_of("[", 0); //, size_type num );
    size_t end = s.find_first_of(",]", 0);
    vector<mReal> f;
    while(start < s.size() && end < s.size() && start != string::npos && end != string::npos) {
        string val = s.substr(start + 1, end - start - 1);
        f.push_back((mReal) strtod(val.c_str(), NULL));
        start = end;
        end = s.find_first_of(",]", start + 1);
    }
    return f;
}

vector<string> YamlReadVectorString(string s)
{
    size_t start = s.find_first_of("[", 0); //, size_type num );
    size_t end = s.find_first_of(",]", 0);
    vector<string> f;
    while(start < s.size() && end < s.size() && start != string::npos && end != string::npos) {
        string val = s.substr(start + 1, end - start - 1);
        f.push_back(val);
        start = end;
        end = s.find_first_of(",]", start + 1);
    }
    return f;
}


#ifdef YAMLCPP_03
vector<mReal> YamlReadVectorM(const YAML::Node &seq)
{
    vector<mReal> f;
    mReal val;
    for(size_t i = 0; i < seq.size(); i++) {
        seq[i] >> val;
        f.push_back(val);
    }

    return f;
}
#endif
// Converts a hexadecimal string to integer
// Returns 0 if not valid
unsigned int xtoi(const char *xs)
{
    unsigned int n;
    sscanf(xs, "%x", &n);
    return n;
}

void WriteYamlDoc(const char *filepath, YAML::Emitter &doc, string sub_dir)
{
    try{
        std::ofstream fout(filepath);
        fout << doc.c_str();
        fout.close();
        M3_INFO("Writing %s\n",filepath);
    }catch(std::exception &e){
        M3_ERR("Caught error while trying to write %s:\n%s\n",filepath,e.what());
    }
}
bool GetRobotConfigPath(vector<string>& vpath,string sub_dir)
{
    vpath.clear();
    if(GetEnvironmentVariable(M3_ROBOT_ENV_VAR, vpath)) {
        for(size_t i = 0; i < vpath.size(); i++)
            vpath[i] += sub_dir;
    }else{
        return false;
    }
    return true;
}

bool GetFileConfigPath(const char *filename,vector<string>& vpath)
{
    string s(filename);
    vpath.clear();
    if(GetRobotConfigPath(vpath)){
        for(size_t i = 0; i < vpath.size(); i++)
            vpath[i] += s;
    }else{
        return false;
    }
    return true;
}

/*void GetYamlParser(const char *filename, YAML::Parser &parser )
{
    string path;
    YAML::Node node;
    vector<string> vpath;
    YAML::Emitter out;
    GetFileConfigPath(filename,vpath);
    for(vector<string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        ifstream fin((*it).c_str());
        if(fin.fail()) { continue;}
        parser.Load(fin);
        while(parser.GetNextDocument(node)) {
            out << node;
        }
        fin.close();
        fin.clear();
    }
    assert(out.good());
    parser.PrintTokens(cout);
    return;
}*/

/*std::auto_ptr<YAML::Node> GetYamlDocs(const char *filename)
{
    YAML::Parser parser;
    YAML::Node docs;
    vector<string> vpath;
    GetFileConfigPath(filename,vpath);
    for(vector<string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        ifstream fin((*it).c_str());
        if(fin.fail()) { continue;}
        parser.Load(fin);
        //parser.GetNextDocument(docs);
        fin.close();
        fin.clear();
    }
    parser.PrintTokens(cout);
    return docs.Clone();
}*/
#ifdef YAMLCPP_03
bool GetYamlStream(const char *filename, YAML::Emitter &out)
{
    string path;
    YAML::Node node;
    YAML::Parser parser;
    vector<string> vpath;
    if(!GetFileConfigPath(filename,vpath)) return false;
    for(vector<string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        ifstream fin((*it).c_str());
        if(fin.fail()) { continue;}
        parser.Load(fin);
        while(parser.GetNextDocument(node)) {
            out << node;
        }
        fin.close();
        fin.clear();
    }
    assert(out.good());
    //parser.PrintTokens(cout);
    return true;
}
#else
bool GetYamlStream(const char *filename, YAML::Emitter &out)
{
    vector<string> vpath;
    if(!GetFileConfigPath(filename,vpath)) return false;
    for(vector<string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        try{
            if(!file_exists((*it).c_str())){
                M3_WARN("%s does not exists, no file or directory.\n",(*it).c_str());
                continue;
            }
            YAML::Node node = YAML::LoadFile((*it).c_str());
            if(!node.IsNull())
                out << node;
        }catch(...){}
    }
    assert(out.good());
    return true;
}
#endif


bool GetYamlDoc(const char* filename, YAML::Node& doc)
{
    std::string ret = GetYamlDoc(filename,doc,NULL);
    return !ret.empty();
}
#ifndef YAMLCPP_03
bool GetAllYamlDocs(const char* filename, std::vector<YAML::Node>& docs )
{
    if(!filename) return false;
    vector<string> vpath;
    if(!GetFileConfigPath(filename,vpath)) return false;
    for(std::vector<std::string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        try{
            const YAML::Node& node = YAML::LoadFile(*it);
            //cout<<endl<<endl<<YAML::Dump(node)<<endl<<endl;
            //cout<<"Adding "<<*it<<endl;
            docs.push_back(node);
        }catch(...){
            continue;
        }
    }
    return !docs.empty();
}
bool GetAllYamlDocs(std::vector<std::string> vpath, std::vector<YAML::Node>& docs )
{
    //assert(filename!=0);
    //vector<string> vpath;
    //GetFileConfigPath(filename,vpath);
    for(std::vector<std::string>::iterator it = vpath.begin(); it != vpath.end(); ++it) {
        try{
            const YAML::Node& node = YAML::LoadFile(*it);
            //cout<<endl<<endl<<YAML::Dump(node)<<endl<<endl;
            //cout<<"Adding "<<*it<<endl;
            docs.push_back(node);
        }catch(...){
            continue;
        }
    }
    return !docs.empty();
}
#endif

std::string GetYamlDoc(const char* filename, YAML::Node& doc, void * )
{
    if(!filename) return std::string();
    
    vector<string> vpath;
    if(!GetFileConfigPath(filename,vpath)) return std::string();

    vector<string> vpath_root;
    
    if(!GetRobotConfigPath(vpath_root)) return std::string();
    if(vpath.size()!=vpath_root.size()) return std::string();
    
    std::vector<std::pair<string,string> > paths;
    for(size_t i=0;i<vpath.size();i++)
        paths.push_back(pair<string,string>(vpath[i],vpath_root[i]));

    bool verbose = paths.size()>1;
    string root_path = std::string();
    string path = std::string();
    for(std::vector<std::pair<string,string> >::reverse_iterator it = paths.rbegin(); it != paths.rend(); ++it) {
        //cout<<"Trying with : "<<it->second<<endl;
        //A.H: Let's start by the very last one i.e a local version.
        //If the file exists, load it, otherwise go to previous path (down to original robot_config)
        //If the file is loaded, then check for an optional find_str provided to checkif this is the right file to load, otherwise go to previous path
        path = it->first;
        root_path = it->second;
#ifndef YAMLCPP_03
        try{
            if(m3rt::file_exists(path))
                doc = YAML::LoadFile(path);
            else
                continue;
        }catch(YAML::Exception){
            continue;
        }
#else
        YAML::Parser parser;
        ifstream fin(path.c_str());
        if(fin.fail()) {
            if(verbose)
                cout<<"Could not read "<<path<<" , trying the next one."<<endl;
            continue;

        }
        parser.Load(fin);
        parser.GetNextDocument(doc);
        fin.close();
        fin.clear();
#endif
        cout << "- Config file: " << path<<endl;
        break;

    }
    return root_path;
}

}
