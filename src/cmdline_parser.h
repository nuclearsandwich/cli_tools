#ifndef CMDLINE_PARSER_H_
#define CMDLINE_PARSER_H_

bool cli_option_exist(char ** begin, char ** end, const std::string & option)
{
  return std::find(begin, end, option) != end;
}

char * cli_get_option(char ** begin, char ** end, const std::string & option)
{
  char ** itr = std::find(begin, end, option);
  if (itr != end && ++itr != end)
  {
    return *itr;
  }
  return NULL;
}

#endif
