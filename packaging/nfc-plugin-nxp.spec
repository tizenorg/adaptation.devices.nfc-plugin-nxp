Name:       nfc-plugin-nxp
Summary:    nfc plugin for nxp chip
Version:    0.0.13
Release:    0
Group:      Network & Connectivity
License:    Apache-2.0
Source0:    %{name}-%{version}.tar.gz
Source1: 	%{name}.manifest
BuildRequires:	cmake

%description
nfc plugin for nxp chip

%prep
%setup -q
cp %{SOURCE1} .

%build
cmake . -DCMAKE_INSTALL_PREFIX=%{_prefix}

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/usr/share/license
cp -af LICENSE.APLv2 %{buildroot}/usr/share/license/%{name}

%make_install

%post
if [ "${USER}" = "root" ]
then
    vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -g 6517 -i -f
else
    vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -f
fi

%files
%manifest nfc-plugin-nxp.manifest
%defattr(-,root,root,-)
/usr/lib/nfc/libnfc-plugin.so
/usr/share/license/%{name}
